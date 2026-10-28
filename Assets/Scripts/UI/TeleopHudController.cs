using System.Collections.Generic;
using System.Text;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using OmniBot.VR.Control;
using OmniBot.VR.Core;
using OmniBot.VR.Recording;
using OmniBot.VR.Telemetry;

namespace OmniBot.VR.UI
{
    /// <summary>
    /// The teleop page of the unified OhhO screen — everything the operator needs
    /// while driving a robot, on one surface:
    ///
    ///   - Connection row (robot IP / port / Connect / Disconnect + status dot)
    ///   - Live telemetry summary (odom pose, velocity, arm joints, control mode)
    ///   - Camera feed (RawImage driven by <see cref="Video.CameraFeedController"/>)
    ///   - Dataset recording controls (Start / Stop&amp;Save / Discard, live timer)
    ///   - Control hints for the active robot profile
    ///   - Back-to-garage navigation
    ///
    /// All subscriptions ride on top of <see cref="ConnectionManager"/>'s baseline
    /// topic set; UI refresh is rate-limited to keep the Quest GPU free for
    /// rendering.
    /// </summary>
    public class TeleopHudController : MonoBehaviour
    {
        [Header("Connection")]
        [SerializeField] private TMP_InputField ipField;
        [SerializeField] private TMP_InputField portField;
        [SerializeField] private Button connectButton;
        [SerializeField] private Button disconnectButton;
        [SerializeField] private Image connectionDot;
        [SerializeField] private TMP_Text connectionStatusText;

        [Header("Telemetry")]
        [SerializeField] private TMP_Text poseText;
        [SerializeField] private TMP_Text velocityText;
        [SerializeField] private TMP_Text armText;
        [SerializeField] private TMP_Text modeText;
        [SerializeField] private TMP_Text telemetryBody;
        [SerializeField] private Button telemetryModeButton;
        [SerializeField] private TMP_Text telemetryModeLabel;

        [Header("Camera")]
        [SerializeField] private Button cycleCameraButton;
        [SerializeField] private Video.CameraFeedController cameraFeed;

        [Header("Recording")]
        [SerializeField] private Button startRecordingButton;
        [SerializeField] private Button stopSaveButton;
        [SerializeField] private Button discardButton;
        [SerializeField] private TMP_Text recordingStatusText;
        [SerializeField] private Image recordingDot;

        [Header("Export (upload episodes to the robot's VR bridge)")]
        [SerializeField] private Button exportButton;
        [SerializeField] private TMP_Text exportStatusText;
        [SerializeField] private int bridgeHttpPort = 8765;

        [Header("Robot / navigation")]
        [SerializeField] private TMP_Text robotNameText;
        [SerializeField] private TMP_Text hintsText;
        [SerializeField] private Button backButton;

        [Header("Safety (OHH-202)")]
        [SerializeField] private Image safetyBanner;
        [SerializeField] private TMP_Text safetyStatusText;
        [SerializeField] private Button estopResetButton;
        [SerializeField] private Button estopConfirmButton;

        // ── State ─────────────────────────────────────────────────────────────
        private App.OhhoVrApp _app;
        private Control.TeleopController _teleop;

        readonly TelemetryHudComposer _composer = new TelemetryHudComposer();
        readonly HashSet<string> _subscribed = new HashSet<string>();
        readonly List<KeyValuePair<string, System.Action<string>>> _profileSubs =
            new List<KeyValuePair<string, System.Action<string>>>();
        readonly StringBuilder _poseSb = new StringBuilder(96);
        readonly StringBuilder _velSb = new StringBuilder(96);
        readonly StringBuilder _armSb = new StringBuilder(96);
        readonly StringBuilder _modeSb = new StringBuilder(64);
        RobotProfile _boundProfile;
        bool _coreSubscribed;
        bool _profileBound;
        char[] _bodyChars = new char[4096];
        char[] _poseChars = new char[128];
        char[] _velChars = new char[128];
        char[] _armChars = new char[128];
        char[] _modeChars = new char[64];
        int _bodyCharsLen = -1;
        int _poseCharsLen = -1;
        int _velCharsLen = -1;
        int _armCharsLen = -1;
        int _modeCharsLen = -1;

        private static readonly Color ConnectedColor = new Color(0.18f, 0.85f, 0.45f);
        private static readonly Color DisconnectedColor = new Color(0.85f, 0.25f, 0.25f);
        private static readonly Color RecordingColor = new Color(1f, 0.2f, 0.2f);
        private static readonly Color IdleDotColor = new Color(0.45f, 0.45f, 0.5f);
        private static readonly Color ArmedColor = new Color(0.12f, 0.45f, 0.28f, 0.96f);
        private static readonly Color LatchedColor = new Color(0.75f, 0.12f, 0.12f, 0.98f);
        private static readonly Color DisarmedColor = new Color(0.55f, 0.38f, 0.08f, 0.96f);
        private static readonly Color SafetyIdleColor = new Color(0.12f, 0.14f, 0.2f, 0.94f);

        private TMP_Text _resetLabel;
        private TMP_Text _confirmLabel;

        // OmniBot catalog names used only when the active profile leaves a field blank.
        // A descriptor-built profile replaces these whenever it names the topic.
        private const string CatalogOdom = "/odom";
        private const string CatalogImu = "/imu/data";
        private const string CatalogJointStates = "/arm/joint_states";
        private const string CatalogControlModeActive = "/control_mode/active";
        private const string CatalogArmModeActive = "/arm/cmd_mode/active";

        // ── Unity lifecycle ───────────────────────────────────────────────────

        private void Awake()
        {
            _app = FindObjectOfType<App.OhhoVrApp>();
            _teleop = FindObjectOfType<Control.TeleopController>();
        }

        private void OnEnable()
        {
            EnsureSafetyWidgets();
            if (connectButton != null) connectButton.onClick.AddListener(OnConnectClicked);
            if (disconnectButton != null) disconnectButton.onClick.AddListener(OnDisconnectClicked);
            if (cycleCameraButton != null) cycleCameraButton.onClick.AddListener(OnCycleCamera);
            if (startRecordingButton != null) startRecordingButton.onClick.AddListener(OnStartRecording);
            if (stopSaveButton != null) stopSaveButton.onClick.AddListener(OnStopSave);
            if (discardButton != null) discardButton.onClick.AddListener(OnDiscard);
            if (exportButton != null) exportButton.onClick.AddListener(OnExportAll);
            if (backButton != null) backButton.onClick.AddListener(OnBack);
            if (estopResetButton != null) estopResetButton.onClick.AddListener(OnEstopResetClicked);
            if (estopConfirmButton != null) estopConfirmButton.onClick.AddListener(OnEstopConfirmClicked);
            EnsureTelemetryWidgets();
            if (telemetryModeButton != null) telemetryModeButton.onClick.AddListener(OnTelemetryMode);

            var conn = ConnectionManager.Instance;
            if (conn != null) conn.OnConnectionStateChanged += OnConnectionState;
            if (_teleop != null) _teleop.TeleopStarted += OnTeleopStarted;

            EnsureSubscriptions();
            RefreshTelemetryUi();

            // Pre-fill the connection fields from the saved preferences.
            if (conn != null)
            {
                if (ipField != null) ipField.text = conn.CurrentIp;
                if (portField != null) portField.text = conn.CurrentPort.ToString();
            }

            RefreshRobotLabel();
            ApplyHudRows();
            OnConnectionState(conn != null && conn.IsConnected);
            RefreshRecordingUi();
        }

        private void OnDisable()
        {
            if (connectButton != null) connectButton.onClick.RemoveListener(OnConnectClicked);
            if (disconnectButton != null) disconnectButton.onClick.RemoveListener(OnDisconnectClicked);
            if (cycleCameraButton != null) cycleCameraButton.onClick.RemoveListener(OnCycleCamera);
            if (startRecordingButton != null) startRecordingButton.onClick.RemoveListener(OnStartRecording);
            if (stopSaveButton != null) stopSaveButton.onClick.RemoveListener(OnStopSave);
            if (discardButton != null) discardButton.onClick.RemoveListener(OnDiscard);
            if (exportButton != null) exportButton.onClick.RemoveListener(OnExportAll);
            if (backButton != null) backButton.onClick.RemoveListener(OnBack);
            if (estopResetButton != null) estopResetButton.onClick.RemoveListener(OnEstopResetClicked);
            if (estopConfirmButton != null) estopConfirmButton.onClick.RemoveListener(OnEstopConfirmClicked);
            if (telemetryModeButton != null) telemetryModeButton.onClick.RemoveListener(OnTelemetryMode);

            var conn = ConnectionManager.Instance;
            if (conn != null) conn.OnConnectionStateChanged -= OnConnectionState;
            if (_teleop != null) _teleop.TeleopStarted -= OnTeleopStarted;
            if (_coreSubscribed || _profileSubs.Count > 0)
            {
                var ros = ROSBridgeClient.Instance;
                DropProfileSubs(ros);
                DropCoreSubs(ros);
            }
            _profileBound = false;
            _boundProfile = null;
        }

        private void OnTeleopStarted(RobotProfile profile)
        {
            ApplyHudRows();
            _profileBound = false;
            EnsureSubscriptions();
            RefreshRobotLabel();
        }

        private void Update()
        {
            double now = Time.realtimeSinceStartupAsDouble;
            if (cameraFeed != null)
            {
                _composer.NoteVideo(
                    cameraFeed.HasVideoFrame,
                    cameraFeed.LastVideoFrameTime,
                    cameraFeed.CurrentCameraTopic,
                    now);
            }
            RefreshSafetyUi();
            ApplyHardCaps();
            if (!_composer.ConsumeFrame(Time.unscaledDeltaTime, now)) return;
            EnsureSubscriptions();
            PaintTelemetry(now);
            RefreshRecordingUi();
        }

        // ── Connection ────────────────────────────────────────────────────────

        private void OnConnectClicked()
        {
            // The session banner and safety gate follow ActiveSession. A manual
            // connect must not swap the robot out from under that label.
            if (OmniBot.VR.Control.Session.ActiveSession.Current != null) return;

            string ip = ipField != null ? ipField.text.Trim() : "192.168.1.101";
            int port = 9090;
            if (portField != null) int.TryParse(portField.text.Trim(), out port);
            if (string.IsNullOrEmpty(ip)) ip = "192.168.1.101";

            SetConnectionLabel("Connecting…");
            ConnectionManager.Instance.Connect(ip, port);
        }

        private void OnDisconnectClicked()
        {
            ConnectionManager.Instance.Disconnect();
        }

        private void OnConnectionState(bool connected)
        {
            if (connectionDot != null)
                connectionDot.color = connected ? ConnectedColor : DisconnectedColor;
            SetConnectionLabel(connected ? "Connected" : "Disconnected");
            bool manualConnect = OmniBot.VR.Control.Session.ActiveSession.Current == null;
            if (ipField != null) ipField.interactable = manualConnect;
            if (portField != null) portField.interactable = manualConnect;
            if (connectButton != null) connectButton.interactable = manualConnect && !connected;
            if (disconnectButton != null) disconnectButton.interactable = connected;
        }

        private void SetConnectionLabel(string s)
        {
            if (connectionStatusText != null) connectionStatusText.text = s;
        }

        // ── Telemetry ─────────────────────────────────────────────────────────

        private void OnTelemetryMode()
        {
            _composer.Expanded = !_composer.Expanded;
            if (telemetryModeLabel != null)
                telemetryModeLabel.text = _composer.Expanded ? "Expanded" : "Compact";
            RefreshTelemetryUi();
        }

        private void ApplyHudRows()
        {
            var profile = _teleop != null ? _teleop.Profile : null;
            if (profile == null) return;
            var rows = TeleopHudPlan.Rows(profile);
            SetRowActive(poseText, TeleopHudPlan.Has(rows, TeleopHudPlan.Pose));
            SetRowActive(velocityText, TeleopHudPlan.Has(rows, TeleopHudPlan.Velocity));
            SetRowActive(armText,
                TeleopHudPlan.Has(rows, TeleopHudPlan.Arm) || TeleopHudPlan.Has(rows, TeleopHudPlan.Gripper));
            SetRowActive(modeText, TeleopHudPlan.Has(rows, TeleopHudPlan.Mode));
        }

        private static void SetRowActive(TMP_Text text, bool active)
        {
            if (text != null) text.gameObject.SetActive(active);
        }

        private void EnsureSubscriptions()
        {
            var ros = ROSBridgeClient.Instance;
            if (ros == null) return;
            if (!_coreSubscribed)
            {
                Sub(ros, TelemetryTopics.Telemetry, TelemetryTopics.StringType, OnTelemetry);
                Sub(ros, TelemetryTopics.SafetyState, TelemetryTopics.StringType, OnSafety);
                Sub(ros, TelemetryTopics.HeartbeatEcho, TelemetryTopics.StringType, OnHeartbeatEcho);
                Sub(ros, CatalogControlModeActive, "std_msgs/String", OnBaseMode);
                Sub(ros, CatalogArmModeActive, "std_msgs/String", OnArmMode);
                _coreSubscribed = true;
            }

            RobotProfile profile = _teleop != null ? _teleop.Profile : null;
            if (_profileBound && ReferenceEquals(profile, _boundProfile)) return;
            BindProfileTopics(ros, profile);
            _boundProfile = profile;
            _profileBound = true;
        }

        private void BindProfileTopics(ROSBridgeClient ros, RobotProfile profile)
        {
            DropProfileSubs(ros);
            RobotTopics topics = profile != null ? profile.Topics : null;
            bool baseDrive = profile != null && profile.BaseDof > 0;
            string odom = topics != null && !string.IsNullOrEmpty(topics.Odom)
                ? topics.Odom
                : (baseDrive ? CatalogOdom : "");
            string imu = topics != null && !string.IsNullOrEmpty(topics.Imu) ? topics.Imu : CatalogImu;
            string joints = topics != null && !string.IsNullOrEmpty(topics.JointStates) ? topics.JointStates : CatalogJointStates;
            SubProfile(ros, odom, "nav_msgs/Odometry", OnOdom);
            SubProfile(ros, imu, "sensor_msgs/Imu", OnImu);
            SubProfile(ros, joints, "sensor_msgs/JointState", OnJoints);
            if (topics == null) return;
            SubProfileDistinct(ros, topics.Telemetry, TelemetryTopics.Telemetry, TelemetryTopics.StringType, OnTelemetry);
            SubProfileDistinct(ros, topics.ControlModeActive, CatalogControlModeActive, "std_msgs/String", OnBaseMode);
            SubProfileDistinct(ros, topics.ArmModeActive, CatalogArmModeActive, "std_msgs/String", OnArmMode);
        }

        private void Sub(ROSBridgeClient ros, string topic, string type, System.Action<string> callback)
        {
            if (ros == null || string.IsNullOrEmpty(topic) || callback == null) return;
            if (!_subscribed.Add(topic)) return;
            ros.Subscribe(topic, type, callback);
        }

        private void SubProfile(ROSBridgeClient ros, string topic, string type, System.Action<string> callback)
        {
            if (ros == null || string.IsNullOrEmpty(topic) || callback == null) return;
            ros.Subscribe(topic, type, callback);
            _profileSubs.Add(new KeyValuePair<string, System.Action<string>>(topic, callback));
        }

        /// <summary>
        /// Listens on a descriptor topic only when it is not the core fallback.
        /// The same callback stays on the core topic, so dropping the profile
        /// subscription cannot remove that listener.
        /// </summary>
        private void SubProfileDistinct(ROSBridgeClient ros, string topic, string fallback, string type, System.Action<string> callback)
        {
            if (string.IsNullOrEmpty(topic) || topic == fallback) return;
            SubProfile(ros, topic, type, callback);
        }

        private void DropProfileSubs(ROSBridgeClient ros)
        {
            if (ros != null)
            {
                for (int i = 0; i < _profileSubs.Count; i++)
                    ros.Unsubscribe(_profileSubs[i].Key, _profileSubs[i].Value);
            }
            _profileSubs.Clear();
        }

        private void DropCoreSubs(ROSBridgeClient ros)
        {
            if (!_coreSubscribed) return;
            Unsub(ros, TelemetryTopics.Telemetry, OnTelemetry);
            Unsub(ros, TelemetryTopics.SafetyState, OnSafety);
            Unsub(ros, TelemetryTopics.HeartbeatEcho, OnHeartbeatEcho);
            Unsub(ros, CatalogControlModeActive, OnBaseMode);
            Unsub(ros, CatalogArmModeActive, OnArmMode);
            _coreSubscribed = false;
        }

        private void Unsub(ROSBridgeClient ros, string topic, System.Action<string> callback)
        {
            if (ros == null || string.IsNullOrEmpty(topic) || callback == null) return;
            ros.Unsubscribe(topic, callback);
            _subscribed.Remove(topic);
        }

        private void OnTelemetry(string json)
        {
            _composer.IngestTelemetry(json, Time.realtimeSinceStartupAsDouble);
        }

        private void OnSafety(string json)
        {
            _composer.IngestSafety(json, Time.realtimeSinceStartupAsDouble);
        }

        private void OnHeartbeatEcho(string json)
        {
            _composer.IngestHeartbeatEcho(json, Time.realtimeSinceStartupAsDouble);
        }

        private void OnBaseMode(string json)
        {
            _composer.NoteRawBaseMode(ReadStringData(json), Time.realtimeSinceStartupAsDouble);
        }

        private void OnArmMode(string json)
        {
            _composer.NoteRawArmMode(ReadStringData(json), Time.realtimeSinceStartupAsDouble);
        }

        private void OnOdom(string json)
        {
            OdometryMsg odom;
            try { odom = JsonConvert.DeserializeObject<OdometryMsg>(json); }
            catch { return; }
            if (odom == null) return;
            bool hasPose = odom.pose != null && odom.pose.pose != null;
            bool hasTwist = odom.twist != null && odom.twist.twist != null;
            double x = 0, y = 0, qx = 0, qy = 0, qz = 0, qw = 1, vx = 0, vy = 0, wz = 0;
            if (hasPose)
            {
                if (odom.pose.pose.position != null)
                {
                    x = odom.pose.pose.position.x;
                    y = odom.pose.pose.position.y;
                }
                if (odom.pose.pose.orientation != null)
                {
                    qx = odom.pose.pose.orientation.x;
                    qy = odom.pose.pose.orientation.y;
                    qz = odom.pose.pose.orientation.z;
                    qw = odom.pose.pose.orientation.w;
                }
            }
            if (hasTwist)
            {
                if (odom.twist.twist.linear != null)
                {
                    vx = odom.twist.twist.linear.x;
                    vy = odom.twist.twist.linear.y;
                }
                if (odom.twist.twist.angular != null)
                    wz = odom.twist.twist.angular.z;
            }
            _composer.NoteRawBase(x, y, qx, qy, qz, qw, vx, vy, wz, hasPose, hasTwist, Time.realtimeSinceStartupAsDouble);
        }

        private void OnImu(string json)
        {
            ImuMsg imu;
            try { imu = JsonConvert.DeserializeObject<ImuMsg>(json); }
            catch { return; }
            if (imu == null || imu.orientation == null) return;
            double ax = imu.linear_acceleration != null ? imu.linear_acceleration.x : 0;
            double ay = imu.linear_acceleration != null ? imu.linear_acceleration.y : 0;
            double az = imu.linear_acceleration != null ? imu.linear_acceleration.z : 0;
            _composer.NoteRawImu(
                imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w,
                ax, ay, az, Time.realtimeSinceStartupAsDouble);
        }

        private void OnJoints(string json)
        {
            JointStateMsg msg;
            try { msg = JsonConvert.DeserializeObject<JointStateMsg>(json); }
            catch { return; }
            if (msg == null || msg.position == null) return;
            double now = Time.realtimeSinceStartupAsDouble;
            _composer.BeginRawJoints(now);
            int n = msg.position.Length;
            if (n > 16) n = 16;
            for (int i = 0; i < n; i++)
            {
                bool hasVel = msg.velocity != null && i < msg.velocity.Length;
                bool hasEff = msg.effort != null && i < msg.effort.Length;
                string name = msg.name != null && i < msg.name.Length ? msg.name[i] : null;
                _composer.AddRawJoint(
                    name,
                    msg.position[i],
                    hasVel ? msg.velocity[i] : 0,
                    hasEff ? msg.effort[i] : 0,
                    hasVel,
                    hasEff);
            }
        }

        static string ReadStringData(string json)
        {
            if (string.IsNullOrEmpty(json)) return null;
            try
            {
                JToken data = JObject.Parse(json)["data"];
                if (data == null || data.Type == JTokenType.Null) return null;
                return data.ToString();
            }
            catch
            {
                return null;
            }
        }

        private void ApplyHardCaps()
        {
            RobotProfile profile = _teleop != null ? _teleop.Profile : null;
            double lin = profile != null && profile.MaxLinVel > 0f ? profile.MaxLinVel : RobotConfig.MAX_LINEAR_VEL;
            double ang = profile != null && profile.MaxAngVel > 0f ? profile.MaxAngVel : RobotConfig.MAX_ANGULAR_VEL;
            _composer.SetHardCaps(lin, ang);
            _composer.SetArmPresent(profile == null || profile.HasArm);
        }

        private void RefreshTelemetryUi()
        {
            double now = Time.realtimeSinceStartupAsDouble;
            ApplyHardCaps();
            _composer.Write(now);
            PaintTelemetry(now);
        }

        private void PaintTelemetry(double now)
        {
            ApplyText(telemetryBody, _composer.Body, ref _bodyChars, ref _bodyCharsLen);
            _composer.WriteLegacy(now, _poseSb, _velSb, _armSb, _modeSb);
            ApplyText(poseText, _poseSb, ref _poseChars, ref _poseCharsLen);
            ApplyText(velocityText, _velSb, ref _velChars, ref _velCharsLen);
            ApplyText(armText, _armSb, ref _armChars, ref _armCharsLen);
            ApplyText(modeText, _modeSb, ref _modeChars, ref _modeCharsLen);
            if (hintsText != null && _teleop != null)
                hintsText.text = _teleop.DriveHint + "\n" + _teleop.ManipHint + "\n" + _teleop.ArmStatus;
        }

        static void ApplyText(TMP_Text label, StringBuilder sb, ref char[] cache, ref int cacheLen)
        {
            if (label == null || sb == null) return;
            int n = sb.Length;
            if (n == cacheLen && Same(sb, cache, n)) return;
            if (cache == null || cache.Length < n)
                cache = new char[n < 256 ? 256 : n];
            sb.CopyTo(0, cache, 0, n);
            cacheLen = n;
            label.richText = true;
            label.SetCharArray(cache, 0, n);
        }

        static bool Same(StringBuilder sb, char[] cache, int n)
        {
            if (cache == null) return false;
            for (int i = 0; i < n; i++)
            {
                if (sb[i] != cache[i]) return false;
            }
            return true;
        }

        private void EnsureTelemetryWidgets()
        {
            if (telemetryBody != null) return;
            var panel = new GameObject("Telemetry V2", typeof(RectTransform));
            panel.transform.SetParent(transform, false);
            var panelRt = (RectTransform)panel.transform;
            panelRt.anchorMin = new Vector2(0.5f, 0.5f);
            panelRt.anchorMax = new Vector2(0.5f, 0.5f);
            panelRt.pivot = new Vector2(0.5f, 0.5f);
            panelRt.sizeDelta = new Vector2(680f, 860f);
            panelRt.anchoredPosition = new Vector2(40f, 0f);
            var bg = panel.AddComponent<Image>();
            bg.color = new Color(0.04f, 0.06f, 0.10f, 0.94f);

            var textGo = new GameObject("Telemetry Body", typeof(RectTransform));
            textGo.transform.SetParent(panel.transform, false);
            var textRt = (RectTransform)textGo.transform;
            textRt.anchorMin = Vector2.zero;
            textRt.anchorMax = Vector2.one;
            textRt.offsetMin = new Vector2(12f, 12f);
            textRt.offsetMax = new Vector2(-12f, -68f);
            telemetryBody = textGo.AddComponent<TextMeshProUGUI>();
            telemetryBody.fontSize = 16f;
            telemetryBody.alignment = TextAlignmentOptions.TopLeft;
            telemetryBody.enableWordWrapping = true;
            telemetryBody.richText = true;
            telemetryBody.color = Color.white;
            telemetryBody.raycastTarget = false;
            if (poseText != null && poseText.font != null) telemetryBody.font = poseText.font;

            var buttonGo = new GameObject("Telemetry Mode", typeof(RectTransform));
            buttonGo.transform.SetParent(panel.transform, false);
            var buttonRt = (RectTransform)buttonGo.transform;
            buttonRt.anchorMin = new Vector2(1f, 1f);
            buttonRt.anchorMax = new Vector2(1f, 1f);
            buttonRt.pivot = new Vector2(1f, 1f);
            buttonRt.sizeDelta = new Vector2(180f, 44f);
            buttonRt.anchoredPosition = new Vector2(-12f, -12f);
            var buttonImage = buttonGo.AddComponent<Image>();
            buttonImage.color = new Color(0f, 0.831f, 1f, 1f);
            telemetryModeButton = buttonGo.AddComponent<Button>();
            telemetryModeButton.targetGraphic = buttonImage;

            var labelGo = new GameObject("Text", typeof(RectTransform));
            labelGo.transform.SetParent(buttonGo.transform, false);
            var labelRt = (RectTransform)labelGo.transform;
            labelRt.anchorMin = Vector2.zero;
            labelRt.anchorMax = Vector2.one;
            labelRt.offsetMin = Vector2.zero;
            labelRt.offsetMax = Vector2.zero;
            telemetryModeLabel = labelGo.AddComponent<TextMeshProUGUI>();
            telemetryModeLabel.text = "Expanded";
            telemetryModeLabel.fontSize = 20f;
            telemetryModeLabel.alignment = TextAlignmentOptions.Center;
            telemetryModeLabel.color = Color.black;
            telemetryModeLabel.raycastTarget = false;
            if (telemetryBody.font != null) telemetryModeLabel.font = telemetryBody.font;
        }

        // ── Camera ────────────────────────────────────────────────────────────

        private void OnCycleCamera()
        {
            if (cameraFeed != null) cameraFeed.CycleCamera();
        }

        // ── Recording ─────────────────────────────────────────────────────────

        private void OnStartRecording()
        {
            EpisodeManager.Instance.StartEpisode();
            RefreshRecordingUi();
        }

        private void OnStopSave()
        {
            EpisodeManager.Instance.StopEpisode();
            RefreshRecordingUi();
        }

        private void OnDiscard()
        {
            EpisodeManager.Instance.DiscardEpisode();
            RefreshRecordingUi();
        }

        private void RefreshRecordingUi()
        {
            var em = EpisodeManager.Instance;
            bool recording = em.State == EpisodeManager.EpisodeState.Recording || em.State == EpisodeManager.EpisodeState.Paused;

            if (recordingDot != null)
                recordingDot.color = recording ? RecordingColor : IdleDotColor;
            if (recordingStatusText != null)
                recordingStatusText.text = recording
                    ? $"REC {em.CurrentEpisodeName}  {em.EpisodeDuration:F0}s"
                    : "Idle";
            if (startRecordingButton != null) startRecordingButton.interactable = !recording;
            if (stopSaveButton != null) stopSaveButton.interactable = recording;
            if (discardButton != null) discardButton.interactable = recording;
        }

        // ── Export (readme §7: upload episodes to the robot's VR bridge) ──────

        private void OnExportAll()
        {
            StartCoroutine(ExportAllEpisodes());
        }

        private System.Collections.IEnumerator ExportAllEpisodes()
        {
            string dir = System.IO.Path.Combine(Application.persistentDataPath, "recordings");
            if (!System.IO.Directory.Exists(dir))
            {
                ShowExportStatus("No recordings yet.");
                yield break;
            }

            string[] files = System.IO.Directory.GetFiles(dir, "*.jsonl");
            if (files.Length == 0)
            {
                ShowExportStatus("No recordings yet.");
                yield break;
            }

            ShowExportStatus($"Exporting {files.Length} episode(s)…");
            int succeeded = 0;
            foreach (string file in files)
            {
                yield return ExportSingleEpisode(file);
                succeeded++;
            }
            ShowExportStatus($"Exported {succeeded}/{files.Length} episodes.");
        }

        private System.Collections.IEnumerator ExportSingleEpisode(string filePath)
        {
            string robotIp = PlayerPrefs.GetString("robot_ip", "192.168.1.101");
            string endpoint = $"http://{robotIp}:{bridgeHttpPort}/upload_episode";

            byte[] fileData;
            try { fileData = System.IO.File.ReadAllBytes(filePath); }
            catch (System.Exception ex)
            {
                ShowExportStatus($"Read failed: {ex.Message}");
                yield break;
            }

            string filename = System.IO.Path.GetFileName(filePath);
            var form = new WWWForm();
            form.AddBinaryData("file", fileData, filename, "application/jsonl");

            using (var req = UnityEngine.Networking.UnityWebRequest.Post(endpoint, form))
            {
                req.timeout = 30;
                yield return req.SendWebRequest();
                if (req.result != UnityEngine.Networking.UnityWebRequest.Result.Success)
                    ShowExportStatus($"Export failed ({filename}): {req.error}");
            }
        }

        private Coroutine _exportStatusRoutine;
        private void ShowExportStatus(string message)
        {
            if (exportStatusText == null) return;
            exportStatusText.text = message;
            exportStatusText.gameObject.SetActive(true);
            if (_exportStatusRoutine != null) StopCoroutine(_exportStatusRoutine);
            _exportStatusRoutine = StartCoroutine(HideExportStatus(4f));
        }

        private System.Collections.IEnumerator HideExportStatus(float delay)
        {
            yield return new WaitForSeconds(delay);
            if (exportStatusText != null) exportStatusText.gameObject.SetActive(false);
        }

        // ── Robot label / hints ───────────────────────────────────────────────

        private void RefreshRobotLabel()
        {
            var profile = _teleop != null ? _teleop.Profile : null;
            var session = OmniBot.VR.Control.Session.ActiveSession.Current;
            if (robotNameText != null)
            {
                if (profile != null && session != null)
                {
                    robotNameText.text = OmniBot.VR.Control.Session.SessionBannerStyle.LabelFor(session.Kind)
                        + "  ·  " + profile.Name;
                    robotNameText.color = OmniBot.VR.Control.Session.SessionBannerStyle.ColorFor(session.Kind);
                }
                else
                    robotNameText.text = profile != null ? profile.Name : "No robot selected";
            }
            if (session != null)
                OmniBot.VR.UI.Session.SessionModeBanner.Apply(gameObject, session.Kind);
            if (hintsText != null && _teleop != null)
                hintsText.text = _teleop.DriveHint + "\n" + _teleop.ManipHint + "\n" + _teleop.ArmStatus;
        }

        // ── Navigation ────────────────────────────────────────────────────────

        private void OnBack()
        {
            if (_app != null) _app.StopTeleopAndReturnToGarage();
        }

        // ── Safety state + e-stop reset ───────────────────────────────────────

        private void OnEstopResetClicked()
        {
            if (_teleop == null) _teleop = FindObjectOfType<Control.TeleopController>();
            _teleop?.RequestEstopReset();
            RefreshSafetyUi();
        }

        private void OnEstopConfirmClicked()
        {
            if (_teleop == null) _teleop = FindObjectOfType<Control.TeleopController>();
            _teleop?.ConfirmEstopReset();
            RefreshSafetyUi();
        }

        private void RefreshSafetyUi()
        {
            var safety = _teleop != null ? _teleop.Safety : null;
            string line = safety != null ? safety.StatusLine : "SAFETY — no session";
            if (safetyStatusText != null)
            {
                safetyStatusText.text = line;
                safetyStatusText.fontStyle = FontStyles.Bold;
            }

            bool latched = safety != null && safety.Latched;
            bool pending = safety != null && safety.ResetConfirmPending;
            bool linked = safety != null && safety.LinkConnected;
            if (safetyBanner != null)
            {
                if (latched) safetyBanner.color = LatchedColor;
                else if (safety != null && safety.StateKnown && safety.Armed) safetyBanner.color = ArmedColor;
                else if (safety != null && safety.StateKnown) safetyBanner.color = DisarmedColor;
                else safetyBanner.color = SafetyIdleColor;
            }

            if (estopResetButton != null) estopResetButton.interactable = latched && linked;
            if (_resetLabel != null) _resetLabel.text = pending ? "Reset armed" : "Reset e-stop";
            if (estopConfirmButton != null)
            {
                estopConfirmButton.gameObject.SetActive(pending);
                estopConfirmButton.interactable = pending && latched && linked;
            }
        }

        /// <summary>
        /// The shipped scene predates the safety strip. Build it when the
        /// serialized widgets were not assigned so the latch is visible without
        /// a scene rebuild.
        /// </summary>
        private void EnsureSafetyWidgets()
        {
            if (safetyStatusText != null && estopResetButton != null && estopConfirmButton != null)
            {
                _resetLabel = estopResetButton.GetComponentInChildren<TMP_Text>();
                _confirmLabel = estopConfirmButton.GetComponentInChildren<TMP_Text>();
                return;
            }

            TMP_FontAsset font = connectionStatusText != null ? connectionStatusText.font : null;
            Sprite uiSprite = connectButton != null ? connectButton.GetComponent<Image>()?.sprite : null;
            if (uiSprite == null && connectionDot != null) uiSprite = connectionDot.sprite;

            var bannerGo = new GameObject("Safety Banner", typeof(RectTransform));
            bannerGo.transform.SetParent(transform, false);
            bannerGo.transform.SetAsLastSibling();
            var bannerRt = (RectTransform)bannerGo.transform;
            bannerRt.anchorMin = new Vector2(0.5f, 1f);
            bannerRt.anchorMax = new Vector2(0.5f, 1f);
            bannerRt.pivot = new Vector2(0.5f, 1f);
            bannerRt.sizeDelta = new Vector2(980f, 88f);
            bannerRt.anchoredPosition = new Vector2(0f, -12f);
            safetyBanner = bannerGo.AddComponent<Image>();
            safetyBanner.sprite = uiSprite;
            safetyBanner.color = SafetyIdleColor;

            safetyStatusText = CreateLabel("Safety Status", bannerGo.transform, font, 28, TextAlignmentOptions.MidlineLeft);
            var statusRt = safetyStatusText.rectTransform;
            statusRt.anchorMin = new Vector2(0f, 0f);
            statusRt.anchorMax = new Vector2(0f, 1f);
            statusRt.pivot = new Vector2(0f, 0.5f);
            statusRt.sizeDelta = new Vector2(560f, 0f);
            statusRt.anchoredPosition = new Vector2(20f, 0f);
            safetyStatusText.text = "SAFETY";

            estopResetButton = CreateHudButton("E-Stop Reset", bannerGo.transform, font, uiSprite, "Reset e-stop", LatchedColor, out _resetLabel);
            PlaceHudButton(estopResetButton, new Vector2(-280f, 0f));
            estopConfirmButton = CreateHudButton("E-Stop Confirm", bannerGo.transform, font, uiSprite, "Confirm reset", new Color(0.95f, 0.85f, 0.2f), out _confirmLabel);
            PlaceHudButton(estopConfirmButton, new Vector2(-40f, 0f));
            if (_confirmLabel != null) _confirmLabel.color = Color.black;
            estopConfirmButton.gameObject.SetActive(false);
        }

        private static TMP_Text CreateLabel(string name, Transform parent, TMP_FontAsset font, float size, TextAlignmentOptions align)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var tmp = go.AddComponent<TextMeshProUGUI>();
            if (font != null) tmp.font = font;
            tmp.fontSize = size;
            tmp.color = Color.white;
            tmp.alignment = align;
            tmp.enableWordWrapping = false;
            tmp.raycastTarget = false;
            return tmp;
        }

        private static Button CreateHudButton(string name, Transform parent, TMP_FontAsset font, Sprite sprite, string label, Color bg, out TMP_Text labelText)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var img = go.AddComponent<Image>();
            img.sprite = sprite;
            img.color = bg;
            var btn = go.AddComponent<Button>();
            btn.targetGraphic = img;
            labelText = CreateLabel("Text", go.transform, font, 22, TextAlignmentOptions.Center);
            var rt = labelText.rectTransform;
            rt.anchorMin = Vector2.zero;
            rt.anchorMax = Vector2.one;
            rt.offsetMin = Vector2.zero;
            rt.offsetMax = Vector2.zero;
            labelText.text = label;
            return btn;
        }

        private static void PlaceHudButton(Button button, Vector2 anchoredPos)
        {
            var rt = (RectTransform)button.transform;
            rt.anchorMin = new Vector2(1f, 0.5f);
            rt.anchorMax = new Vector2(1f, 0.5f);
            rt.pivot = new Vector2(1f, 0.5f);
            rt.sizeDelta = new Vector2(220f, 56f);
            rt.anchoredPosition = anchoredPos;
        }
    }
}
