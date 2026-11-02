using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using NativeWebSocket;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using OmniBot.VR.Telemetry;

namespace OmniBot.VR.Core
{
    /// <summary>
    /// Thread-safe ROSBridge v2 WebSocket client.
    /// Dispatches incoming messages on the Unity main thread via ConcurrentQueue.
    /// Implements exponential back-off auto-reconnect (2s, 4s, 8s, max 3 attempts).
    /// </summary>
    public class ROSBridgeClient : MonoBehaviour
    {
        // ── Singleton ────────────────────────────────────────────────────────
        private static ROSBridgeClient _instance;
        public static bool HasInstance => _instance != null;

        public static ROSBridgeClient Instance
        {
            get
            {
                if (_instance == null)
                {
                    GameObject go = new GameObject("ROSBridgeClient");
                    DontDestroyOnLoad(go);
                    _instance = go.AddComponent<ROSBridgeClient>();
                }
                return _instance;
            }
        }

        // ── Public state ─────────────────────────────────────────────────────
        public bool IsConnected => _socket != null && _socket.State == WebSocketState.Open;
        public string Url { get; private set; }

        // ── Events ───────────────────────────────────────────────────────────
        public event Action OnConnected;
        public event Action OnDisconnected;
        public event Action<string> OnError;

        // ── Private fields ───────────────────────────────────────────────────
        private WebSocket _socket;
        private readonly ConcurrentQueue<(string topic, string json)> _incomingQueue
            = new ConcurrentQueue<(string, string)>();
        private readonly Dictionary<string, List<Action<string>>> _subscribers
            = new Dictionary<string, List<Action<string>>>();
        private readonly Dictionary<string, string> _subscribeWire
            = new Dictionary<string, string>();
        private readonly object _subscribersLock = new object();
        private readonly ConcurrentDictionary<string, Action<bool, string>> _calls
            = new ConcurrentDictionary<string, Action<bool, string>>();
        private int _callSeq;

        private bool _intentionalDisconnect = false;
        private readonly ReconnectBackoff _backoff = new ReconnectBackoff();
        private float _reconnectTimer = 0f;
        private bool _reconnecting = false;
        private float _reconnectDelay = 0f;
        private int _serviceId;

        // ── Unity lifecycle ──────────────────────────────────────────────────
        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(gameObject);
                return;
            }
            _instance = this;
            DontDestroyOnLoad(gameObject);
        }

        private void Update()
        {
            // NativeWebSocket requires DispatchMessageQueue() on the main thread
            // (no-op on WebGL, required on all other platforms).
            if (_socket != null)
            {
                _socket.DispatchMessageQueue();
            }

            // Drain the incoming message queue on the main thread
            while (_incomingQueue.TryDequeue(out var item))
            {
                DispatchIncoming(item.topic, item.json);
            }

            // Auto-reconnect logic
            if (_reconnecting && !IsConnected)
            {
                _reconnectTimer -= Time.deltaTime;
                if (_reconnectTimer <= 0f)
                {
                    _reconnecting = false;
                    if (!string.IsNullOrEmpty(Url))
                    {
                        Debug.Log($"[ROSBridgeClient] Reconnect attempt {_backoff.Attempts}/{ReconnectBackoff.MaxAttempts} to {Url}");
                        _ = ConnectAsync(Url);
                    }
                }
            }
        }

        private void OnDestroy()
        {
            _intentionalDisconnect = true;
            Disconnect();
        }

        // ── Public API ───────────────────────────────────────────────────────

        /// <summary>Connects to the given ROSBridge WebSocket URL.</summary>
        public async void Connect(string url)
        {
            _intentionalDisconnect = false;
            _backoff.Reset();
            Url = url;
            await ConnectAsync(url);
        }

        /// <summary>Cleanly disconnects from ROSBridge.</summary>
        public void Disconnect()
        {
            _ = DisconnectAsync();
        }

        /// <summary>
        /// Closes the current socket and completes when that close finishes.
        /// <see cref="Disconnect"/> does not wait, so a session switch must await this.
        /// </summary>
        public System.Threading.Tasks.Task DisconnectAsync()
        {
            _intentionalDisconnect = true;
            _reconnecting = false;
            FailPendingCalls("Rosbridge disconnected.");
            var socket = _socket;
            _socket = null;
            return CloseSocketAsync(socket);
        }

        /// <summary>
        /// Advertises a publisher topic so ROSBridge creates the ROS publisher.
        /// Must be called before publishing to a new topic.
        /// </summary>
        public void Advertise(string topic, string type)
        {
            var msg = new ROSAdvertiseMessage(topic, type);
            SendJson(JsonConvert.SerializeObject(msg));
        }

        /// <summary>
        /// Fired after a message has been handed to the socket. Not fired when the
        /// publish is dropped because the socket is closed. Observers must not
        /// change connection, drive, or e-stop behavior; exceptions are logged
        /// and swallowed so a metric cannot block the send.
        /// </summary>
        public static event Action<string, object> MessagePublished;

        /// <summary>Serializes and publishes a message to a ROS topic.</summary>
        public void Publish<T>(string topic, T msg)
        {
            if (!IsConnected)
            {
                Debug.LogWarning($"[ROSBridgeClient] Publish dropped — not connected. Topic: {topic}");
                return;
            }
            NoteOutgoing(msg);
            var wrapper = new ROSPublishMessage<T>(topic, msg);
            SendJson(JsonConvert.SerializeObject(wrapper));
            NotifyPublished(topic, msg);
        }

        private static void NotifyPublished(string topic, object msg)
        {
            var handler = MessagePublished;
            if (handler == null) return;
            try
            {
                handler.Invoke(topic, msg);
            }
            catch (Exception ex)
            {
                Debug.LogError($"[ROSBridgeClient] MessagePublished observer failed: {ex.Message}");
            }
        }

        /// <summary>
        /// Display-only echo of twist and joint targets that are about to go out.
        /// Does not change the message, the 20 Hz publish, or any safety default.
        /// </summary>
        static void NoteOutgoing<T>(T msg)
        {
            double now = Time.realtimeSinceStartupAsDouble;
            if (msg is TwistMsg twist && twist.linear != null && twist.angular != null)
            {
                CommandEcho.Shared.NoteTwist(twist.linear.x, twist.linear.y, twist.angular.z, now);
                return;
            }
            if (msg is JointStateMsg joints && joints.position != null)
                CommandEcho.Shared.NoteJoints(joints.position, joints.position.Length, now);
        }

        /// <summary>
        /// Subscribes to a ROS topic. The callback is invoked on the main thread
        /// with the raw JSON string of the msg field. A png-compressed publish
        /// is unwrapped before the callback runs.
        /// </summary>
        public void Subscribe(string topic, string type, Action<string> callback)
        {
            Subscribe(topic, type, callback, 0, null);
        }

        /// <summary>
        /// Subscribes with a rosbridge throttle (milliseconds) and optional
        /// compression (<c>png</c>). The wire options are taken from the first
        /// subscriber on that topic and are replayed after a reconnect.
        /// </summary>
        public void Subscribe(string topic, string type, Action<string> callback, int throttleRateMs, string compression)
        {
            if (string.IsNullOrEmpty(topic) || callback == null) return;
            lock (_subscribersLock)
            {
                if (!_subscribers.TryGetValue(topic, out var callbacks))
                {
                    callbacks = new List<Action<string>>();
                    _subscribers[topic] = callbacks;
                    var subMsg = new ROSSubscribeMessage(topic, type, throttleRateMs, compression);
                    string wire = JsonConvert.SerializeObject(subMsg);
                    _subscribeWire[topic] = wire;
                    SendJson(wire);
                }
                else if (callbacks.Contains(callback))
                {
                    return;
                }

                callbacks.Add(callback);
            }
        }

        /// <summary>Unsubscribes all callbacks for a topic.</summary>
        public void Unsubscribe(string topic)
        {
            lock (_subscribersLock)
            {
                _subscribers.Remove(topic);
                _subscribeWire.Remove(topic);
            }
            if (string.IsNullOrEmpty(topic)) return;
            var unsubMsg = new ROSUnsubscribeMessage(topic);
            SendJson(JsonConvert.SerializeObject(unsubMsg));
        }

        /// <summary>
        /// rosbridge <c>call_service</c>. The second argument is a service type
        /// (heartbeat and e-stop reset; empty args, <c>ohho_svc_</c> id) or a JSON
        /// argument object (connection preflight passes "{}"). The callback runs
        /// on the main thread. This does not publish a motion command.
        /// </summary>
        public void CallService(string service, string typeOrArgsJson, Action<bool, string> callback)
        {
            if (string.IsNullOrEmpty(service))
            {
                callback?.Invoke(false, "Service name is empty.");
                return;
            }
            if (!IsConnected)
            {
                callback?.Invoke(false, "Rosbridge is not connected.");
                return;
            }

            bool argsCall = IsJsonArgs(typeOrArgsJson);
            string id = argsCall
                ? "ohho-" + Interlocked.Increment(ref _callSeq).ToString(System.Globalization.CultureInfo.InvariantCulture)
                : "ohho_svc_" + (++_serviceId);
            if (callback != null)
                _calls[id] = callback;

            if (!argsCall)
            {
                SendJson(RosBridgeServiceCall.FormatRequest(id, service, typeOrArgsJson));
                return;
            }

            JObject args;
            try
            {
                args = string.IsNullOrEmpty(typeOrArgsJson) ? new JObject() : JObject.Parse(typeOrArgsJson);
            }
            catch (Exception)
            {
                args = new JObject();
            }

            var root = new JObject
            {
                ["op"] = "call_service",
                ["id"] = id,
                ["service"] = service,
                ["args"] = args,
            };
            SendJson(root.ToString(Formatting.None));
        }

        private static bool IsJsonArgs(string value)
        {
            if (string.IsNullOrWhiteSpace(value)) return true;
            char first = value.TrimStart()[0];
            return first == '{' || first == '[';
        }

        /// <summary>
        /// Removes one callback. The rosbridge unsubscribe is sent only when
        /// that was the last listener, so a HUD refresh cannot tear down the
        /// recorder or another panel sharing the topic.
        /// </summary>
        public void Unsubscribe(string topic, Action<string> callback)
        {
            if (string.IsNullOrEmpty(topic) || callback == null) return;
            bool sendUnsub = false;
            lock (_subscribersLock)
            {
                if (!_subscribers.TryGetValue(topic, out var list)) return;
                list.RemoveAll(existing => existing != null && existing.Equals(callback));
                if (list.Count > 0) return;
                _subscribers.Remove(topic);
                _subscribeWire.Remove(topic);
                sendUnsub = true;
            }
            if (sendUnsub)
                SendJson(JsonConvert.SerializeObject(new ROSUnsubscribeMessage(topic)));
        }

        /// <summary>Unsubscribes every topic this client is listening to.</summary>
        public void UnsubscribeAll()
        {
            List<string> topics;
            lock (_subscribersLock)
            {
                topics = new List<string>(_subscribers.Keys);
            }

            foreach (var topic in topics)
                Unsubscribe(topic);
        }

        // ── Private helpers ──────────────────────────────────────────────────

        private async System.Threading.Tasks.Task CloseSocketAsync(WebSocket socket)
        {
            if (socket == null) return;
            socket.OnOpen -= HandleOpen;
            socket.OnClose -= HandleClose;
            socket.OnError -= HandleError;
            socket.OnMessage -= HandleMessage;
            if (socket.State != WebSocketState.Open)
                return;
            try
            {
                await socket.Close();
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ROSBridgeClient] Socket close failed: {ex.Message}");
            }
        }

        private async System.Threading.Tasks.Task ConnectAsync(string url)
        {
            // Close the previous socket before replacing it. Detaching the
            // handlers alone leaves that connection open.
            var previous = _socket;
            _socket = null;
            if (previous != null)
                await CloseSocketAsync(previous);
            if (_intentionalDisconnect)
                return;

            _socket = new WebSocket(url);
            _socket.OnOpen += HandleOpen;
            _socket.OnClose += HandleClose;
            _socket.OnError += HandleError;
            _socket.OnMessage += HandleMessage;

            try
            {
                await _socket.Connect();
            }
            catch (Exception ex)
            {
                Debug.LogError($"[ROSBridgeClient] Connect exception: {ex.Message}");
                ScheduleReconnect();
            }
        }

        private void HandleOpen()
        {
            Debug.Log($"[ROSBridgeClient] Connected to {Url}");
            _backoff.Reset();
            _reconnecting = false;
            // Dispatch on main thread via queue trick — enqueue a sentinel
            _incomingQueue.Enqueue(("__connected__", ""));
        }

        private void HandleClose(WebSocketCloseCode code)
        {
            Debug.Log($"[ROSBridgeClient] Disconnected. Code: {code}");
            _incomingQueue.Enqueue(("__disconnected__", ""));
            if (!_intentionalDisconnect)
            {
                ScheduleReconnect();
            }
        }

        private void HandleError(string errorMsg)
        {
            Debug.LogError($"[ROSBridgeClient] WebSocket error: {errorMsg}");
            _incomingQueue.Enqueue(("__error__", errorMsg));
        }

        private void HandleMessage(byte[] data)
        {
            string json = System.Text.Encoding.UTF8.GetString(data);
            try
            {
                JObject root = JObject.Parse(json);
                string op = root["op"]?.ToString();
                if (op == "service_response" || (op == "status" && IsPendingCall(root)))
                {
                    _incomingQueue.Enqueue(("__service__", json));
                    return;
                }
                if (!RosbridgeIncoming.TryExtractPublish(json, out string topic, out string msgJson))
                    return;
                _incomingQueue.Enqueue((topic, msgJson));
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ROSBridgeClient] Failed to parse incoming message: {ex.Message}");
            }
        }

        private void DispatchIncoming(string topic, string json)
        {
            // Handle sentinel events on main thread
            if (topic == "__connected__")
            {
                // Re-subscribe to all known topics after reconnect
                ResubscribeAll();
                OnConnected?.Invoke();
                return;
            }
            if (topic == "__disconnected__")
            {
                OnDisconnected?.Invoke();
                return;
            }
            if (topic == "__error__")
            {
                OnError?.Invoke(json);
                return;
            }
            if (topic == "__service__")
            {
                DispatchService(json);
                return;
            }

            List<Action<string>> callbacks = null;
            lock (_subscribersLock)
            {
                if (_subscribers.TryGetValue(topic, out var list))
                {
                    callbacks = new List<Action<string>>(list);
                }
            }

            if (callbacks != null)
            {
                foreach (var cb in callbacks)
                {
                    try { cb(json); }
                    catch (Exception ex)
                    {
                        Debug.LogError($"[ROSBridgeClient] Subscriber callback exception on {topic}: {ex.Message}");
                    }
                }
            }
        }

        private bool IsPendingCall(JObject root)
        {
            string id = root["id"]?.ToString();
            return !string.IsNullOrEmpty(id) && _calls.ContainsKey(id);
        }

        private void DispatchService(string json)
        {
            JObject root;
            try { root = JObject.Parse(json); }
            catch (Exception) { return; }

            string id = root["id"]?.ToString();
            if (string.IsNullOrEmpty(id) || !_calls.TryRemove(id, out var callback) || callback == null)
                return;

            string op = root["op"]?.ToString();
            if (op == "status")
            {
                InvokeCall(callback, false, root["msg"]?.ToString() ?? "rosbridge status error");
                return;
            }

            if (!RosBridgeServiceCall.TryParseResponse(json, out _, out bool ok, out string values))
            {
                Debug.LogWarning("[ROSBridgeClient] Unreadable service_response.");
                InvokeCall(callback, false, "Unreadable service_response.");
                return;
            }

            // Preflight treats a missing values object as "{}". Trigger responses
            // keep values.success as the outcome via RosBridgeServiceCall.
            if (string.IsNullOrEmpty(values)) values = "{}";
            InvokeCall(callback, ok, values);
        }

        private void FailPendingCalls(string reason)
        {
            var ids = new List<string>(_calls.Keys);
            for (int i = 0; i < ids.Count; i++)
            {
                if (_calls.TryRemove(ids[i], out var callback) && callback != null)
                    InvokeCall(callback, false, reason);
            }
        }

        private static void InvokeCall(Action<bool, string> callback, bool ok, string body)
        {
            try { callback(ok, body); }
            catch (Exception ex)
            {
                Debug.LogError($"[ROSBridgeClient] Service callback exception: {ex.Message}");
            }
        }

        private void ResubscribeAll()
        {
            lock (_subscribersLock)
            {
                foreach (var kvp in _subscribers)
                {
                    if (_subscribeWire.TryGetValue(kvp.Key, out string wire))
                        SendJson(wire);
                    else
                        SendJson(JsonConvert.SerializeObject(new ROSSubscribeMessage(kvp.Key, "")));
                }
            }
        }

        private void ScheduleReconnect()
        {
            if (_intentionalDisconnect) return;
            if (!_backoff.TrySchedule(out float delay))
            {
                Debug.LogWarning("[ROSBridgeClient] Max reconnect attempts reached.");
                OnError?.Invoke("Max reconnect attempts reached.");
                return;
            }
            // Exponential back-off: 2s, 4s, 8s
            _reconnectDelay = delay;
            _reconnectTimer = _reconnectDelay;
            _reconnecting = true;
            Debug.Log($"[ROSBridgeClient] Scheduling reconnect in {_reconnectDelay}s (attempt {_backoff.Attempts})");
        }

        private async void SendJson(string json)
        {
            if (_socket == null || _socket.State != WebSocketState.Open)
            {
                Debug.LogWarning($"[ROSBridgeClient] SendJson dropped — socket not open.");
                return;
            }
            try
            {
                await _socket.SendText(json);
            }
            catch (Exception ex)
            {
                Debug.LogError($"[ROSBridgeClient] SendJson exception: {ex.Message}");
            }
        }
    }
}
