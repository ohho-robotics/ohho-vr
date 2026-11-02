using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using OmniBot.VR.Control;
using OmniBot.VR.Control.Session;

namespace OmniBot.VR.Video
{
    /// <summary>
    /// Profile-driven camera feed. A sim session opens the curved sim screen on
    /// the chase camera, with a strip for front / wrist / BEV and a wrist
    /// picture-in-picture. A real session keeps the profile camera order on the
    /// flat panel. WebRTC is preferred when the topic maps to an ohho_webrtc
    /// track; otherwise MJPEG runs with off-thread decode. Right thumbstick
    /// press cycles cameras.
    /// </summary>
    public class CameraFeedController : MonoBehaviour
    {
        [Header("Display")]
        [SerializeField] private RawImage displayImage;
        [SerializeField] private TMP_Text cameraNameOverlay;
        [SerializeField] private Texture2D noSignalTexture;

        [Header("Transport")]
        [Tooltip("Prefer WebRTC when com.unity.webrtc can run. Falls back to MJPEG per camera.")]
        [SerializeField] private bool preferWebRtc = true;

        [Tooltip("Optional label that shows WebRTC or MJPEG. When empty, the camera name overlay includes the transport.")]
        [SerializeField] private TMP_Text transportLabel;

        private SimScreenPlan _plan;
        private SimScreenPresenter _presenter;
        private int _mainIndex;
        private IVideoSource _mainSource;
        private MjpegVideoSource _mainMjpeg;
        private MjpegVideoSource _pipMjpeg;
        private string _robotIp;
        private bool _configured;
        private bool _offline;
        private bool _ipHooked;
        private bool _inFallback;
        private bool _hasFrame;
        private double _frameReceivedAt;
        private float _statsTimer;
        private int _generation;
        private readonly StreamFpsCounter _fps = new StreamFpsCounter();
        private Texture2D _grey;

        /// <summary>Active transport, "WebRTC" or "MJPEG", or null when there is no camera.</summary>
        public string ActiveTransport { get; private set; }

        /// <summary>Fires when the HUD transport label changes.</summary>
        public event Action<string> OnTransportChanged;

        /// <summary>True after at least one decoded frame on the current camera.</summary>
        public bool HasVideoFrame => _hasFrame;

        /// <summary>Realtime clock of the last decoded frame. Video age is now minus this.</summary>
        public double LastVideoFrameTime => _frameReceivedAt;

        /// <summary>ROS topic of the camera currently selected, or null.</summary>
        public string CurrentCameraTopic
        {
            get
            {
                if (_plan == null || _plan.Views == null || _plan.Views.Count == 0) return null;
                int idx = _mainIndex;
                if (idx < 0) idx = 0;
                if (idx >= _plan.Views.Count) idx = _plan.Views.Count - 1;
                return _plan.Views[idx].Topic;
            }
        }

        private void Awake()
        {
            if (string.IsNullOrEmpty(_robotIp))
                _robotIp = PlayerPrefs.GetString("robot_ip", "192.168.1.101");
            EnsurePresenter();
            HookIp();
        }

        private void OnEnable()
        {
            HookIp();
            if (_configured) RestartAll();
        }

        private void OnDisable()
        {
            StopAll();
        }

        private void OnDestroy()
        {
            StopAll();
            var conn = OmniBot.VR.Core.ConnectionManager.Instance;
            if (conn != null && _ipHooked) conn.OnRobotIpChanged -= OnRobotIpChanged;
            _ipHooked = false;
            if (_grey != null) Destroy(_grey);
        }

        private void Update()
        {
            _mainSource?.Tick();
            _pipMjpeg?.Tick();
            if (OVRInput.GetDown(OVRInput.Button.SecondaryThumbstick, OVRInput.Controller.RTouch))
                CycleCamera();
            _statsTimer += Time.unscaledDeltaTime;
            if (_statsTimer >= 0.1f)
            {
                _statsTimer = 0f;
                RefreshStats();
            }
        }

        /// <summary>Image topics from the profile. Empty when the descriptor lists none.</summary>
        public static List<string> TopicsFor(RobotProfile profile)
        {
            if (profile?.Topics?.Images == null || profile.Topics.Images.Count == 0)
                return new List<string>();
            return profile.Topics.Images;
        }

        /// <summary>
        /// Configure from the selected robot. Chase is the default when the
        /// active session is <see cref="SessionKind.Sim"/>. Session video URLs
        /// and the offline flag come from <see cref="ActiveSession"/>.
        /// </summary>
        public void Configure(RobotProfile profile)
        {
            if (string.IsNullOrEmpty(_robotIp))
                _robotIp = PlayerPrefs.GetString("robot_ip", "192.168.1.101");
            var endpoint = ActiveSession.Current;
            _offline = endpoint != null && endpoint.Offline;
            SessionKind? kind = null;
            string host = _robotIp;
            IList<string> urls = null;
            if (endpoint != null)
            {
                kind = endpoint.Kind;
                if (!string.IsNullOrEmpty(endpoint.Host)) host = endpoint.Host;
                urls = endpoint.VideoUrls;
            }
            IList<string> topics = TopicsFor(profile);
            _plan = SimScreenPlan.Build(kind, topics, host, urls);
            _mainIndex = _plan.Views.Count == 0 ? 0 : _plan.DefaultIndex;
            _configured = true;
            _hasFrame = false;
            _fps.Clear();
            EnsurePresenter();
            _presenter.Apply(_plan);
            _presenter.SetSelection(_mainIndex);
            ApplyLabel();
            if (isActiveAndEnabled) RestartAll();
        }

        public void CycleCamera()
        {
            if (_plan == null || _plan.Views.Count == 0) return;
            SetCamera((_mainIndex + 1) % _plan.Views.Count);
        }

        public void SetCamera(int index)
        {
            if (_plan == null || _plan.Views.Count == 0) return;
            int next = Mathf.Clamp(index, 0, _plan.Views.Count - 1);
            if (next == _mainIndex && _mainSource != null) return;
            int oldPip = _plan.PipIndexFor(_mainIndex);
            _mainIndex = next;
            int newPip = _plan.PipIndexFor(_mainIndex);
            string topic = _plan.Views[_mainIndex].Topic;
            ApplyLabel();
            if (_presenter != null) _presenter.SetSelection(_mainIndex);
            if (!isActiveAndEnabled || _offline) return;

            if (_mainSource is WebRtcVideoSource live && live.TryShow(topic))
            {
                ShowTransport(topic, live.TransportName);
                if (oldPip != newPip)
                {
                    StopPip();
                    StartPip();
                }
                else if (_presenter != null)
                    _presenter.SetPipVisible(newPip >= 0);
                RefreshStats();
                return;
            }

            _hasFrame = false;
            _fps.Clear();
            StopMain();
            StartMain();
            if (oldPip != newPip)
            {
                StopPip();
                StartPip();
            }
            else if (_presenter != null)
                _presenter.SetPipVisible(newPip >= 0);
            RefreshStats();
        }

        private void EnsurePresenter()
        {
            if (_presenter != null) return;
            _presenter = GetComponent<SimScreenPresenter>();
            if (_presenter == null) _presenter = gameObject.AddComponent<SimScreenPresenter>();
            _presenter.Bind(this);
        }

        private void HookIp()
        {
            if (_ipHooked) return;
            var conn = OmniBot.VR.Core.ConnectionManager.Instance;
            if (conn == null) return;
            conn.OnRobotIpChanged += OnRobotIpChanged;
            _ipHooked = true;
        }

        private void OnRobotIpChanged(string newIp)
        {
            _robotIp = newIp;
            if (_plan != null) _plan.RetargetHost(newIp);
            if (_configured && isActiveAndEnabled) RestartAll();
        }

        private void RestartAll()
        {
            StopAll();
            if (!_configured || _plan == null) return;
            ApplyLabel();
            if (_offline || _plan.Views.Count == 0)
            {
                ShowNoSignal();
                ClearTransport();
                return;
            }
            _hasFrame = false;
            _fps.Clear();
            StartMain();
            StartPip();
            if (_presenter != null) _presenter.SetSelection(_mainIndex);
            RefreshStats();
        }

        private void StartMain()
        {
            if (_plan == null || _plan.Views.Count == 0) return;
            _generation++;
            int generation = _generation;
            string topic = _plan.Views[_mainIndex].Topic;
            var topics = new List<string>();
            for (int i = 0; i < _plan.Views.Count; i++)
            {
                if (!string.IsNullOrEmpty(_plan.Views[i].Topic))
                    topics.Add(_plan.Views[i].Topic);
            }
            List<string> trackNames = WebRtcTracks.NamesForTopics(topics);
            string currentName = WebRtcTracks.NameForTopic(topic);
            string host = string.IsNullOrEmpty(_plan.Host) ? _robotIp : _plan.Host;
            bool wantWebRtc = VideoTransportChoice.Select(preferWebRtc, WebRtcVideoSource.IsAvailable)
                    == VideoTransportChoice.WebRtc
                && currentName != null
                && trackNames.Count > 0;

            if (wantWebRtc)
            {
                var webrtc = new WebRtcVideoSource(this);
                webrtc.OnFrame += OnMainFrame;
                webrtc.OnError += err =>
                {
                    Debug.LogWarning("[CameraFeedController] WebRTC error: " + err);
                    if (generation != _generation) return;
                    FallbackToMjpeg();
                };
                _mainSource = webrtc;
                _mainMjpeg = null;
                ShowTransport(topic, webrtc.TransportName);
                webrtc.StartTracks(host, trackNames);
                if (_mainSource == webrtc)
                    webrtc.TryShow(topic);
                Debug.Log("[CameraFeedController] Starting WebRTC for " + string.Join(",", trackNames));
                return;
            }
            StartMainMjpeg();
        }

        private void FallbackToMjpeg()
        {
            if (_inFallback) return;
            _inFallback = true;
            _generation++;
            Debug.Log("[CameraFeedController] Falling back to MJPEG");
            StopMain();
            StartMainMjpeg();
            _inFallback = false;
        }

        private void StartMainMjpeg()
        {
            if (_plan == null || _plan.Views.Count == 0) return;
            string topic = _plan.Views[_mainIndex].Topic;
            string url = _plan.UrlFor(_mainIndex, SimScreenCameras.MainQuality, SimScreenCameras.MainWidth);
            if (string.IsNullOrEmpty(url))
            {
                ShowNoSignal();
                ClearTransport();
                return;
            }
            var mjpeg = new MjpegVideoSource(this);
            mjpeg.JpegQuality = SimScreenCameras.MainQuality;
            mjpeg.JpegWidth = SimScreenCameras.MainWidth;
            mjpeg.OnFrame += OnMainFrame;
            mjpeg.OnError += err =>
            {
                Debug.LogWarning("[CameraFeedController] MJPEG " + err);
                ShowNoSignal();
            };
            mjpeg.StartUrl(url);
            _mainSource = mjpeg;
            _mainMjpeg = mjpeg;
            ShowTransport(topic, mjpeg.TransportName);
            Debug.Log("[CameraFeedController] MJPEG " + _plan.Views[_mainIndex].Label + " " + url);
        }

        private void StartPip()
        {
            if (_plan == null) return;
            int pip = _plan.PipIndexFor(_mainIndex);
            if (_presenter != null) _presenter.SetPipVisible(pip >= 0);
            if (pip < 0) return;
            string url = _plan.UrlFor(pip, SimScreenCameras.PipQuality, SimScreenCameras.PipWidth);
            if (string.IsNullOrEmpty(url)) return;
            var mjpeg = new MjpegVideoSource(this);
            mjpeg.JpegQuality = SimScreenCameras.PipQuality;
            mjpeg.JpegWidth = SimScreenCameras.PipWidth;
            mjpeg.OnFrame += OnPipFrame;
            mjpeg.OnError += err =>
            {
                Debug.LogWarning("[CameraFeedController] Wrist PiP " + err);
                if (_presenter != null) _presenter.SetPipVisible(false);
            };
            mjpeg.StartUrl(url);
            _pipMjpeg = mjpeg;
        }

        private void StopAll()
        {
            StopMain();
            StopPip();
        }

        private void StopMain()
        {
            if (_mainSource != null)
            {
                _mainSource.OnFrame -= OnMainFrame;
                _mainSource.Dispose();
                _mainSource = null;
            }
            _mainMjpeg = null;
        }

        private void StopPip()
        {
            if (_pipMjpeg != null)
            {
                _pipMjpeg.OnFrame -= OnPipFrame;
                _pipMjpeg.Dispose();
                _pipMjpeg = null;
            }
            if (_presenter != null) _presenter.SetPipVisible(false);
        }

        private void OnMainFrame(Texture2D frame)
        {
            _hasFrame = true;
            double now = Time.realtimeSinceStartupAsDouble;
            _fps.Mark(now);
            _frameReceivedAt = _mainMjpeg != null && _mainMjpeg.HasPresentedFrame
                ? _mainMjpeg.LastFrameReceivedAt
                : now;
            if (displayImage != null)
            {
                displayImage.color = Color.white;
                displayImage.texture = frame;
            }
            if (_presenter != null) _presenter.SetMainTexture(frame);
            RefreshStats();
        }

        private void OnPipFrame(Texture2D frame)
        {
            if (_presenter != null) _presenter.SetPipTexture(frame);
        }

        private void ShowTransport(string topic, string transport)
        {
            ActiveTransport = transport;
            if (transportLabel != null) transportLabel.text = transport ?? "";
            ApplyLabel();
            if (transportLabel == null && cameraNameOverlay != null && !string.IsNullOrEmpty(transport))
            {
                string label = cameraNameOverlay.text;
                cameraNameOverlay.text = string.IsNullOrEmpty(label)
                    ? VideoHud.Label(topic, transport)
                    : label + "  " + transport;
            }
            OnTransportChanged?.Invoke(transport);
        }

        private void ClearTransport()
        {
            ActiveTransport = null;
            if (transportLabel != null) transportLabel.text = "";
            OnTransportChanged?.Invoke(null);
        }

        private void ShowNoSignal()
        {
            _hasFrame = false;
            Texture tex = noSignalTexture != null ? (Texture)noSignalTexture : Grey();
            if (displayImage != null)
            {
                displayImage.color = new Color(0.02f, 0.03f, 0.06f, 1f);
                displayImage.texture = tex;
            }
            if (_presenter != null)
            {
                _presenter.SetMainTexture(null);
                _presenter.SetStats(StreamOverlay.Format(0f, 0f, false));
            }
        }

        private void ApplyLabel()
        {
            string label;
            if (_offline) label = "In-headset offline";
            else if (_plan == null || _plan.Views.Count == 0) label = "No cameras";
            else label = _plan.Views[Mathf.Clamp(_mainIndex, 0, _plan.Views.Count - 1)].Label;
            if (cameraNameOverlay != null) cameraNameOverlay.text = label;
        }

        private void RefreshStats()
        {
            if (_presenter == null) return;
            double now = Time.realtimeSinceStartupAsDouble;
            float age = _hasFrame ? StreamOverlay.AgeMilliseconds(_frameReceivedAt, now) : 0f;
            _presenter.SetStats(StreamOverlay.Format(_fps.Fps(now), age, _hasFrame));
        }

        private Texture2D Grey()
        {
            if (_grey != null) return _grey;
            _grey = new Texture2D(4, 4, TextureFormat.RGB24, false, false);
            var pixels = new Color[16];
            for (int i = 0; i < pixels.Length; i++) pixels[i] = new Color(0.2f, 0.2f, 0.2f);
            _grey.SetPixels(pixels);
            _grey.Apply(false, false);
            return _grey;
        }
    }
}
