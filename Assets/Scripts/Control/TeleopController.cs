using System;
using UnityEngine;
using OmniBot.VR.Comfort;
using OmniBot.VR.Core;
using OmniBot.VR.Control.Preflight;
using OmniBot.VR.Input;
using OmniBot.VR.UI;
using OmniBot.VR.Video;
using OmniBot.VR.Recording;
using OmniBot.VR.Control.Drive;
using OmniBot.VR.Control.Manip;
using OmniBot.VR.Control.Safety;
using OmniBot.VR.Latency;
using OmniBot.VR.Nav;

namespace OmniBot.VR.Control
{
    /// <summary>
    /// The teleop control layer entry point — bridges VR input (OVR controllers +
    /// hand tracking) to the profile-driven <see cref="IDriveScheme"/> +
    /// <see cref="IManipulationScheme"/> selected for the robot the user picked in
    /// their garage. This is the Phase 2 spine: put on the headset, sign in, pick
    /// a robot, and <see cref="StartTeleop"/> wires its profile into the schemes;
    /// every frame after, this controller pumps thumbsticks → drive and hand
    /// poses → manipulation, publishing to the robot over the
    /// <see cref="IRobotLink"/>.
    ///
    /// Replaces the archived BaseController and HandTrackingArmController
    /// (Archive/legacy-control) as the single teleop GameObject.
    /// Those MonoBehaviours were hardcoded to OmniBot; this one is agnostic — the
    /// profile decides what gets driven and how.
    /// </summary>
    public class TeleopController : MonoBehaviour
    {
        [Header("Input")]
        [Tooltip("Hand tracking provider (assign the scene GestureDetector, or auto-found).")]
        [SerializeField] private GestureDetector gestureDetector;

        [Tooltip("Transform at the robot arm's base in the MR scene. Hand position is measured relative to this.")]
        [SerializeField] private Transform handWorkspaceOrigin;

        [Header("Link")]
        [Tooltip("Robot transport. Defaults to the ROSBridge singleton (RosBridgeLink).")]
        [SerializeField] private MonoBehaviour linkProvider;

        [Header("Phase 4 — video + recording")]
        [Tooltip("Profile-driven camera feed. Configure() is called on StartTeleop.")]
        [SerializeField] private CameraFeedController cameraFeed;
        [Tooltip("Legacy MJPEG viewer. Configure() replaces its camera list with the profile topics.")]
        [SerializeField] private CameraFeedViewer cameraFeedViewer;
        [Tooltip("Profile-driven dataset recorder. Configure() is called on StartTeleop.")]
        [SerializeField] private ProfileDrivenRecorder recorder;

        /// <summary>Raised after the profile is applied so the HUD can subscribe to its topics.</summary>
        public event Action<RobotProfile> TeleopStarted;

        private IRobotLink _link;
        private IRobotLink Link => _link ?? (_link = ResolveLink());

        private IRobotLink ResolveLink()
        {
            if (linkProvider is IRobotLink provider) return provider;
            return RosBridgeLink.Instance;
        }

        // ── Active control state ────────────────────────────────────────────────
        public RobotProfile Profile { get; private set; }
        private IDriveScheme _drive;
        private IManipulationScheme _manip;
        private bool _teleopActive;
        private readonly TeleopSafetySession _safety = new TeleopSafetySession();
        private bool _hmdMounted = true;
        private bool _appPaused;
        private bool _appFocused = true;
        private bool _inputFocused = true;

        /// <summary>Heartbeat, safety latch, and post-reconnect deadman hold.</summary>
        public TeleopSafetySession Safety => _safety;

        /// <summary>
        /// Latch copied off the drive scheme in <see cref="StopTeleop"/> so the
        /// next <see cref="StartTeleop"/> can put it back on the same endpoint.
        /// A different connection does not inherit it. This is not keyed by
        /// <see cref="RobotProfile.RobotId"/>, which is the hardware model id.
        /// </summary>
        private bool _emergencyStopLatched;

        /// <summary>ip:port the latch was taken on. Empty when no ConnectionManager is loaded.</summary>
        private string _emergencyStopEndpoint;

        /// <summary>
        /// The latched stop never reached the robot. The replacement scheme
        /// publishes <c>emergency_stop</c> true on the next connected tick.
        /// </summary>
        private bool _emergencyStopPublishPending;

        /// <summary>When set, replaces the ConnectionManager endpoint (edit-mode tests).</summary>
        private string _endpointKeyOverride;

        private ConnectionManager _connection;

        /// <summary>One-line drive control hint (for onboarding / HUD).</summary>
        public string DriveHint => _drive?.HudHint ?? "No drive on this robot";
        /// <summary>One-line manipulation control hint (for onboarding / HUD).</summary>
        public string ManipHint => _manip?.HudHint ?? "No arm on this robot";
        /// <summary>Live arm-gate status (enable, REAL confirm, clutch, jog).</summary>
        public string ArmStatus => _manip?.StatusText ?? "";

        // ── Thumbs-up gesture state (ported from HandTrackingArmController) ──────
        private bool _wasThumbsUp;
        private float _thumbsUpHoldTimer;
        private const float ThumbsUpHoldRequired = 0.5f;
        private const float ClutchTriggerThreshold = 0.6f;

        private bool _originOwned;
        private GameObject _ownedOrigin;
        private WorkspaceSphereVisual _workspaceVisual;

        // ── Button edges, and a HUD e-stop press held until the scheme latches ─
        private bool _prevBothGrips;
        private bool _prevEmergencyChord;
        private bool _hudEmergencyStop;
        // Policy hears true on the latch edge and false only after that latch clears.
        private bool _policyEstopReported;
        // Fixed-base arms have no drive scheme. The same Reset + deadman sequence
        // owns this latch, and it publishes emergency_stop on its own.
        private bool _armEStopLatched;
        private bool _armEStopController;
        private bool _armEStopResetAccepted;
        private bool _armEStopDeadmanReleased;
        private bool _armEStopPublishPending;
        private bool _ownsLatencyCapture;
        private TeleopInput _lastInput;
        private SimServeAerialClient _aerialClient;
        // Set when the operator drops out. Aerial sim-serve does not use the
        // robot-link deadman latch, so the fresh grip press is tracked here.
        private bool _aerialMotionHeld;
        private bool _aerialDeadmanReleased;

        // ── Unity lifecycle ──────────────────────────────────────────────────────
        private void OnEnable()
        {
            var conn = ConnectionManager.Instance;
            if (conn != null) conn.OnConnectionStateChanged += OnConnectionChanged;
            OVRManager.HMDUnmounted += OnHmdUnmounted;
            OVRManager.HMDMounted += OnHmdMounted;
            OVRManager.InputFocusLost += OnInputFocusLost;
            OVRManager.InputFocusAcquired += OnInputFocusAcquired;
        }

        private void OnDisable()
        {
            var conn = ConnectionManager.Instance;
            if (conn != null) conn.OnConnectionStateChanged -= OnConnectionChanged;
            OVRManager.HMDUnmounted -= OnHmdUnmounted;
            OVRManager.HMDMounted -= OnHmdMounted;
            OVRManager.InputFocusLost -= OnInputFocusLost;
            OVRManager.InputFocusAcquired -= OnInputFocusAcquired;
        }

        private void Start()
        {
            if (gestureDetector == null)
                gestureDetector = FindObjectOfType<GestureDetector>();
            _ownsLatencyCapture = LatencyCapture.TryStartFromEnvironment();
            if (_ownsLatencyCapture)
                LatencyCaptureHooks.EnsureCommandAckSubscription();
        }

        private void OnDestroy()
        {
            if (_ownsLatencyCapture)
                LatencyCapture.StopAndWrite();
        }

        private void OnApplicationPause(bool paused)
        {
            _appPaused = paused;
            PushPresence(paused ? "app pause" : "app resume");
        }

        private void OnApplicationFocus(bool focused)
        {
            _appFocused = focused;
            PushPresence(focused ? "focus acquired" : "focus lost");
        }

        private void OnHmdUnmounted()
        {
            _hmdMounted = false;
            PushPresence("hmd unmounted");
        }

        private void OnHmdMounted()
        {
            _hmdMounted = true;
            PushPresence("hmd mounted");
        }

        private void OnInputFocusLost()
        {
            _inputFocused = false;
            PushPresence("focus lost");
        }

        private void OnInputFocusAcquired()
        {
            _inputFocused = true;
            PushPresence("focus acquired");
        }

        private void PushPresence(string reason)
        {
            bool present = _hmdMounted && !_appPaused && _appFocused && _inputFocused;
            IRobotLink link = _teleopActive ? Link : null;
            if (!_safety.SetOperatorPresent(present, reason, link)) return;

            _aerialMotionHeld = true;
            _aerialDeadmanReleased = false;
            // Aerial stop publishes on sim-serve and does not need the robot link.
            // Ground stop and the arm still wait until that link can take a command.
            if (_drive is AerialDriveScheme aerial)
                aerial.Stop(link);
            if (link != null && link.IsConnected)
            {
                if (!(_drive is AerialDriveScheme))
                    _drive?.Stop(link);
                _manip?.Stop(link);
            }
        }

        private void OnConnectionChanged(bool connected)
        {
            if (!connected || !_teleopActive || Profile == null) return;
            AdvertiseTopics(Profile);
            ConfigureSurfaces(Profile);
            TeleopStarted?.Invoke(Profile);
        }

        private void Update()
        {
            if (!_teleopActive) return;

            // A HUD reconnect to another ip:port must not keep this scheme blocked.
            ReleaseLatchIfEndpointChanged();

            bool linkUp = Link != null && Link.IsConnected;
            bool aerialScheme = _drive is AerialDriveScheme;
            bool aerialUp = aerialScheme && _aerialClient != null && _aerialClient.IsConnected;
            // Sample before ReadInput, and only while a link that can move is up.
            // Do not return here: heartbeat, the safety latch, and the deadman
            // still tick when the robot link is down.
            if ((linkUp || aerialUp) && LatencyCapture.IsEnabled)
                LatencyCapture.MarkInputSample();

            var input = ReadInput();
            PumpFrame(input, linkUp, aerialUp);
        }

        /// <summary>
        /// One control frame after input is sampled. Edit-mode tests call this
        /// with a crafted sample so a dropped link still has to latch e-stop.
        /// </summary>
        private void PumpFrame(TeleopInput input, bool linkUp, bool aerialUp)
        {
            if (_hudEmergencyStop)
                input.HudEmergencyStop = true;
            _lastInput = input;

            // Both grips, A+B, and the HUD button latch e-stop. A policy yields
            // on those edges. Releasing the grips does not clear it. The false
            // edge is published only after the latch itself clears.
            NotifyEmergencyStopEdges(input);

            // Heartbeat, latched e-stop, and the ground deadman always run.
            // Aerial is omitted from this tick: safety only drives while the
            // robot link is up, and sim-serve can be connected without it.
            // A fixed-base latch is applied first so the arm samples the stop
            // that this frame just took or cleared.
            bool aerialScheme = _drive is AerialDriveScheme;
            if (!aerialScheme && !(_drive is DriveSchemeBase))
                TickArmOnlyEStop(ref input);
            IDriveScheme groundDrive = aerialScheme ? null : _drive;
            _safety.Tick(Link, ref input, groundDrive, _manip, TickManipulation);
            if (_safety.ConsumeLinkRestored())
                AdvertiseTopics(Profile);

            if (aerialScheme)
            {
                if (linkUp || aerialUp)
                {
                    // Either grip unsticks sticks after a presence drop. The ground
                    // deadman stays the left grip; this hold is aerial only.
                    AerialDriveScheme.AdvancePostLossHold(
                        ref _aerialMotionHeld,
                        ref _aerialDeadmanReleased,
                        input,
                        _safety.OperatorPresent,
                        _safety.Latched);

                    var aerialInput = input;
                    if (!_safety.OperatorPresent || _safety.Latched || _aerialMotionHeld)
                    {
                        // Grip deadman and both-grip e-stop still reach the scheme.
                        // Stick axes stay at zero until presence is back and the
                        // deadman has been released and pressed again.
                        aerialInput.LeftStick = Vector2.zero;
                        aerialInput.RightStick = Vector2.zero;
                    }

                    _drive.Tick(ref aerialInput, Link);
                }
                else if (_drive is AerialDriveScheme aerial)
                {
                    // Both links are down. ReadInput still advances the A+B edge,
                    // so this frame has to latch or a press-and-release is lost.
                    aerial.LatchStopWhileLinksDown(ref input, Link);
                }
            }

            if (_hudEmergencyStop && IsGripEStopActive)
                _hudEmergencyStop = false;

            RememberLatchEndpoint();
            PublishPolicyEStopFromLatch();
        }

        private TeleopInput ReadInput()
        {
            Vector2 leftStick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch);
            Vector2 rightStick = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick, OVRInput.Controller.RTouch);

            bool leftGrip = OVRInput.Get(OVRInput.Button.PrimaryHandTrigger, OVRInput.Controller.LTouch) ||
                            OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.LTouch) > 0.8f;
            bool rightGrip = OVRInput.Get(OVRInput.Button.SecondaryHandTrigger, OVRInput.Controller.RTouch) ||
                             OVRInput.Get(OVRInput.Axis1D.SecondaryHandTrigger, OVRInput.Controller.RTouch) > 0.8f;
            bool bothGrips = leftGrip && rightGrip;
            BothGripsHeld = bothGrips;
            AnyGripHeld = leftGrip || rightGrip;
            // A and B together latch e-stop. Right trigger is the arm clutch and,
            // in a kind=sim session only, base turbo. Left X starts the clear.
            // Grip release must not clear it: the left grip is the deadman.
            bool buttonA = OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.RTouch);
            bool buttonB = OVRInput.Get(OVRInput.Button.Two, OVRInput.Controller.RTouch);
            bool rightTrigger = OVRInput.Get(OVRInput.Button.SecondaryIndexTrigger, OVRInput.Controller.RTouch) ||
                                OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTrigger, OVRInput.Controller.RTouch) > 0.8f;
            bool emergencyChord = buttonA && buttonB;
            bool resetEStop = OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.LTouch);

            var input = new TeleopInput
            {
                LeftStick = leftStick,
                RightStick = rightStick,
                LeftGrip = leftGrip,
                RightGrip = rightGrip,
                RightTrigger = rightTrigger,
                ButtonA = buttonA,
                ButtonB = buttonB,
                BothGrips = bothGrips,
                BothGripsPressed = bothGrips && !_prevBothGrips,
                BothGripsReleased = !bothGrips && _prevBothGrips,
                EmergencyStopPressed = emergencyChord && !_prevEmergencyChord,
                HudEmergencyStop = _hudEmergencyStop,
                ResetEmergencyStop = resetEStop,
                DeltaTime = Time.deltaTime,
            };
            input = ComfortControlMapping.Apply(
                input,
                ComfortSettingsStore.Current.LeftHandedControls);
            _prevBothGrips = bothGrips;
            _prevEmergencyChord = emergencyChord;
            return input;
        }

        private void TickManipulation()
        {
            if (_manip == null) return;

            // Runs without a GestureDetector so controller jog and the enable
            // button still work when hand tracking is absent. The safety session
            // only calls this while motion is allowed. Aerial sim-serve can be
            // up while the robot link is down; arm commands wait for the link.
            if (Link == null || !Link.IsConnected) return;

            HandPose rightHand = default;
            HandPose leftHand = default;
            bool trackingGood = false;
            if (gestureDetector != null)
            {
                rightHand = BuildHandPose(gestureDetector.rightHand);
                leftHand = BuildHandPose(gestureDetector.leftHand);
                trackingGood = rightHand.Quality == HandTrackingQuality.Good;
                TickThumbsUpToggle(gestureDetector.leftHand);
            }

            bool clutch = false;
            bool xDown = false;
            bool cycleJoint = false;
            try
            {
                clutch = OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTrigger, OVRInput.Controller.RTouch) >= ClutchTriggerThreshold
                    || OVRInput.Get(OVRInput.Button.SecondaryIndexTrigger, OVRInput.Controller.RTouch);
                xDown = OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.LTouch);
                cycleJoint = OVRInput.GetDown(OVRInput.Button.PrimaryThumbstick, OVRInput.Controller.LTouch);
            }
            catch (Exception)
            {
                // Editor and headless runs have no OVR plugin. The arm e-stop
                // below still has to be applied from the sampled input.
            }
            bool thumbsPulse = _thumbsUpPulse;
            _thumbsUpPulse = false;

            bool driveLatched = IsGripEStopActive;
            var armInput = new ArmOperatorInput
            {
                ClutchHeld = clutch,
                TrackingGood = trackingGood,
                JogAxis = _lastInput.RightStick.y,
                CycleJointPressed = cycleJoint,
                EmergencyStop = ArmMotionBlocked(_lastInput, driveLatched),
                DeltaTime = _lastInput.DeltaTime,
            };
            // Left X confirms a pending REAL enable. A thumbs-up never does.
            // Either control disarms immediately, and either requests enable
            // when the arm is off. One press cannot both request and confirm.
            var phase = _manip.EnablePhase;
            if (phase == ArmEnablePhase.AwaitingRealConfirmation && xDown)
                armInput.ConfirmRequested = true;
            else if (phase == ArmEnablePhase.Armed && (xDown || thumbsPulse))
                armInput.DisarmRequested = true;
            else if (phase == ArmEnablePhase.Off && (xDown || thumbsPulse))
                armInput.EnableRequested = true;

            _manip.FeedArmInput(armInput);
            _manip.Tick(ref rightHand, ref leftHand, Link);

            if (_workspaceVisual != null)
                _workspaceVisual.SetAppearance(_manip.EnablePhase, _manip.ClutchEngaged, trackingGood);
        }

        private bool _thumbsUpPulse;

        // ── Public API (called by GaragePanelController / OhhoVrApp) ──────────────

        /// <summary>
        /// Start teleoperating the selected robot. Builds the schemes from the
        /// profile, advertises the topics, and begins pumping input every frame.
        /// </summary>
        public void StartTeleop(RobotProfile profile)
        {
            if (profile == null) { Debug.LogWarning("[TeleopController] StartTeleop called with null profile."); return; }

            // Preflight holds real robots until safety checks pass. Stop publishes
            // zero velocity through the existing drive path and does not clear an
            // e-stop, the deadman, or a speed cap.
            if (PreflightGate.BlocksMotion)
            {
                StopTeleop();
                Debug.LogWarning("[TeleopController] Real teleop stays stopped until connection preflight is clear.");
                return;
            }

            StopTeleop();

            Profile = profile;
            if (profile.HasArm)
                EnsureWorkspaceOrigin(profile);
            _drive = ControlSchemeFactory.CreateDriveScheme(profile);
            // Drive schemes bind nav state in Configure. A robot with no drive
            // scheme still must not keep the previous robot's map and goal.
            if (_drive == null)
                NavGoalSession.Active.BeginRobot(profile);
            if (_emergencyStopLatched)
            {
                if (SameEndpoint(_emergencyStopEndpoint, CurrentEndpointKey()))
                {
                    if (_drive is DriveSchemeBase drive)
                        drive.RestoreEmergencyStopLatch(_emergencyStopPublishPending);
                    else
                        RestoreArmEmergencyStop(_emergencyStopPublishPending);
                }
                else
                {
                    _emergencyStopLatched = false;
                    _emergencyStopEndpoint = null;
                    _emergencyStopPublishPending = false;
                    NavGoalSession.Active.SetHeadsetBlock(false, null);
                }
            }
            AttachAerialClient(_drive);
            _manip = ControlSchemeFactory.CreateManipulationScheme(profile, handWorkspaceOrigin);
            if (profile.HasArm)
                EnsureWorkspaceVisual(profile);

            AdvertiseTopics(profile);
            _teleopActive = true;
            PushPresence("session start");
            _safety.Start(Link);
            ConfigureSurfaces(profile);

            Debug.Log($"[TeleopController] Teleop started — {profile.Name} ({profile.Drive}, arm {(profile.HasArm ? $"{profile.ArmDof}-DOF" : "none")}), IK={profile.IkLocation}.");
            Raise(TeleopSessionStarted, profile);
            TeleopStarted?.Invoke(profile);
            PublishPolicyEStopFromLatch();
        }

        private void ConfigureSurfaces(RobotProfile profile)
        {
            if (cameraFeed != null) cameraFeed.Configure(profile);
            if (cameraFeedViewer == null)
                cameraFeedViewer = FindObjectOfType<CameraFeedViewer>();
            if (cameraFeedViewer != null) cameraFeedViewer.Configure(profile);
            if (recorder != null) recorder.Configure(profile);
            EpisodeManager.Instance.Configure(profile);
        }

        /// <summary>Stop teleoperation: zero velocity, disarm arm, drop the schemes.</summary>
        public void StopTeleop()
        {
            if (_drive is DriveSchemeBase drive)
            {
                if (drive.IsEmergencyStopActive)
                {
                    // Keep the endpoint captured when the latch engaged. Reading
                    // it only here would attach the latch to a connection the
                    // operator already switched away from.
                    if (!_emergencyStopLatched || _emergencyStopEndpoint == null)
                        _emergencyStopEndpoint = CurrentEndpointKey();
                    _emergencyStopLatched = true;
                    // Read before Stop. A drop may have latched without a publish.
                    _emergencyStopPublishPending = drive.EmergencyStopPublishPending;
                }
                else
                {
                    _emergencyStopLatched = false;
                    _emergencyStopEndpoint = null;
                    _emergencyStopPublishPending = false;
                }
            }
            else if (_teleopActive)
            {
                // The arm latch lives on this controller. Copy it before the
                // cleanup below clears the live flags. A second stop, after
                // teleop is already inactive, leaves the saved latch alone.
                if (_armEStopLatched)
                {
                    if (!_emergencyStopLatched || _emergencyStopEndpoint == null)
                        _emergencyStopEndpoint = CurrentEndpointKey();
                    _emergencyStopLatched = true;
                    _emergencyStopPublishPending = _armEStopPublishPending;
                }
                else
                {
                    _emergencyStopLatched = false;
                    _emergencyStopEndpoint = null;
                    _emergencyStopPublishPending = false;
                }
            }
            if (!_teleopActive && _drive == null && _manip == null) return;
            bool aerial = _drive is AerialDriveScheme;
            _teleopActive = false;
            _aerialMotionHeld = false;
            _aerialDeadmanReleased = false;
            _safety.Stop(Link);
            _drive?.Stop(Link);
            if (aerial && _aerialClient != null) _aerialClient.Disconnect();
            _manip?.Stop(Link);
            // Hand a running policy back while this profile is still set.
            Raise(TeleopSessionStopped);
            _drive = null;
            _manip = null;
            Profile = null;
            _wasThumbsUp = false;
            _thumbsUpHoldTimer = 0f;
            _prevBothGrips = false;
            _prevEmergencyChord = false;
            _hudEmergencyStop = false;
            _armEStopLatched = false;
            _armEStopController = false;
            _armEStopResetAccepted = false;
            _armEStopDeadmanReleased = false;
            _armEStopPublishPending = false;
            if (LatencyCapture.IsEnabled)
                LatencyCapture.ClearInputSample();
            _thumbsUpPulse = false;
            if (_workspaceVisual != null)
            {
                DestroyOwned(_workspaceVisual.gameObject);
                _workspaceVisual = null;
            }
            if (_originOwned && _ownedOrigin != null)
            {
                DestroyOwned(_ownedOrigin);
                _ownedOrigin = null;
                handWorkspaceOrigin = null;
                _originOwned = false;
            }
        }

        /// <summary>
        /// Edit-mode tests stop teleop. <see cref="Object.Destroy"/> is play mode only.
        /// </summary>
        private void DestroyOwned(UnityEngine.Object obj)
        {
            if (obj == null) return;
            if (Application.isPlaying) Destroy(obj);
            else DestroyImmediate(obj);
        }

        private static void Raise(Action action)
        {
            try { action?.Invoke(); }
            catch (Exception ex) { Debug.LogException(ex); }
        }

        private static void Raise<T>(Action<T> action, T arg)
        {
            try { action?.Invoke(arg); }
            catch (Exception ex) { Debug.LogException(ex); }
        }

        /// <summary>Whether a robot is currently being teleoperated.</summary>
        public bool IsTeleopActive => _teleopActive;

        /// <summary>Both grips are down this frame (the grip e-stop input).</summary>
        public bool BothGripsHeld { get; private set; }

        /// <summary>Either grip is down this frame. Display-only deadman readout.</summary>
        public bool AnyGripHeld { get; private set; }

        /// <summary>
        /// A controller e-stop is holding the drive scheme, or a fixed-base arm
        /// that has no drive scheme.
        /// </summary>
        public bool IsGripEStopActive =>
            (_drive is DriveSchemeBase scheme && scheme.IsEmergencyStopActive) || _armEStopLatched;

        /// <summary>Same transport the drive and arm schemes publish on.</summary>
        public IRobotLink RobotLink => Link;

        /// <summary>
        /// E-stop edge for a running policy. True for both grips, A+B, and the HUD button.
        /// False only after that latch has cleared through Reset and a deadman re-press.
        /// </summary>
        public event Action<bool> EmergencyStopChanged;

        private void NotifyEmergencyStopEdges(TeleopInput input)
        {
            if (input.BothGripsPressed || input.EmergencyStopPressed || input.HudEmergencyStop)
                RaisePolicyEStop(true);
        }

        /// <summary>
        /// Keep the policy latch aligned with the drive or fixed-base latch.
        /// A grip release while that latch is still held does not publish false,
        /// so <c>control_mode=vla</c> cannot start under a latched stop.
        /// </summary>
        private void PublishPolicyEStopFromLatch()
        {
            RaisePolicyEStop(IsGripEStopActive || OperateSafetyLatch.UiEStopLatched);
        }

        private void RaisePolicyEStop(bool latched)
        {
            if (latched == _policyEstopReported) return;
            _policyEstopReported = latched;
            Raise(EmergencyStopChanged, latched);
        }

        /// <summary>Fired after a profile is applied and topics are advertised.</summary>
        public event Action<RobotProfile> TeleopSessionStarted;

        /// <summary>Fired after the schemes stop. A running policy must hand back first.</summary>
        public event Action TeleopSessionStopped;

        /// <summary>
        /// HUD e-stop. Latches until Reset plus a deadman release and re-press.
        /// Releasing the button does not clear it. The request sticks until a
        /// connected tick accepts it. A running policy yields on this call.
        /// </summary>
        public void RequestHudEmergencyStop()
        {
            _hudEmergencyStop = true;
            RaisePolicyEStop(true);
        }

        /// <summary>
        /// Arm motion stops for every e-stop source, for the whole drive latch.
        /// Both grips, A+B, and the HUD button all count. The latch covers the
        /// frames after a one-shot HUD press has been consumed.
        /// </summary>
        public static bool ArmMotionBlocked(in TeleopInput input, bool driveLatched)
        {
            if (driveLatched || input.BothGrips || input.HudEmergencyStop || input.EmergencyStopPressed)
                return true;
            return input.ButtonA && input.ButtonB;
        }

        /// <summary>
        /// Point the live feed at the operate sim slot. Stops the previous feed
        /// so a hidden side screen does not keep streaming.
        /// </summary>
        public void AttachCameraFeed(CameraFeedController feed)
        {
            if (feed == null) return;
            if (cameraFeed != null && cameraFeed != feed) cameraFeed.StopFeed();
            cameraFeed = feed;
            if (Profile != null) feed.Configure(Profile);
        }

        /// <summary>
        /// Publish a control-mode string. Refuses while an e-stop is latched so
        /// nav2 or policy cannot start under a stop.
        /// </summary>
        public void RequestOperateMode(string mode)
        {
            if (!OperateSafetyRules.ModePublishAllowed(OperateSafetyLatch.UiEStopLatched, IsGripEStopActive))
                return;
            if (string.IsNullOrEmpty(mode) || Profile?.Topics == null) return;
            if (string.IsNullOrEmpty(Profile.Topics.ControlMode) || !Link.IsConnected) return;
            Link.Publish(Profile.Topics.ControlMode, new StringMsg(mode));
        }

        /// <summary>Latch e-stop from the operate bar and zero velocity.</summary>
        public void LatchOperateEStop()
        {
            OperateSafetyLatch.Latch();
            RaisePolicyEStop(true);
            if (_drive is DriveSchemeBase scheme)
                scheme.HoldNavForLatchedStop(Link);
            else
                NavGoalSession.Active.SetHeadsetBlock(true, Link);
            if (Profile?.Topics == null || !Link.IsConnected) return;
            if (!string.IsNullOrEmpty(Profile.Topics.EmergencyStop))
                Link.Publish(Profile.Topics.EmergencyStop, new BoolMsg(true));
            if (!string.IsNullOrEmpty(Profile.Topics.CmdVel))
                Link.Publish(Profile.Topics.CmdVel, new TwistMsg(new Vector3Msg(0f, 0f, 0f), new Vector3Msg(0f, 0f, 0f)));
        }

        /// <summary>
        /// Clear the operate-bar latch. Does nothing while both grips are held,
        /// and does not publish a mode change. A controller stop (A+B, the HUD
        /// button, or both grips) stays latched until Reset plus a deadman
        /// release and re-press, including while that deadman is still held.
        /// </summary>
        public bool TryResetOperateEStop()
        {
            if (BothGripsHeld) return false;
            bool schemeStopped = IsGripEStopActive;
            bool wasLatched = OperateSafetyLatch.UiEStopLatched || schemeStopped;
            OperateSafetyLatch.Clear();
            if (!wasLatched)
            {
                PublishPolicyEStopFromLatch();
                return true;
            }
            if (_drive is DriveSchemeBase scheme)
                scheme.ReleaseUiStop(Link, gripHeld: false);
            else if (_armEStopController)
            {
                // The operate bar is clear. The arm latch stays until the
                // deadman sequence in TickArmOnlyEStop.
            }
            else if (_armEStopLatched)
                ArmClearEStop(Link);
            else if (Profile?.Topics != null && Link.IsConnected)
            {
                if (!string.IsNullOrEmpty(Profile.Topics.EmergencyStop))
                    Link.Publish(Profile.Topics.EmergencyStop, new BoolMsg(false));
                if (!string.IsNullOrEmpty(Profile.Topics.CmdVel))
                    Link.Publish(Profile.Topics.CmdVel, new TwistMsg(new Vector3Msg(0f, 0f, 0f), new Vector3Msg(0f, 0f, 0f)));
                NavGoalSession.Active.SetHeadsetBlock(false, null);
            }
            else
                NavGoalSession.Active.SetHeadsetBlock(false, null);
            PublishPolicyEStopFromLatch();
            return true;
        }

        /// <summary>First tap arms the confirm. Second tap calls <c>/ohho/estop_reset</c>.</summary>
        public void RequestEstopReset() => _safety.RequestEstopReset();

        /// <summary>Confirm the HUD e-stop reset and call the robot service.</summary>
        public bool ConfirmEstopReset() => _safety.ConfirmEstopReset(Link);

        /// <summary>
        /// ROSBridge endpoint the latch is scoped to. ConnectionManager ip:port,
        /// or the edit-mode override. Not the hardware model id.
        /// </summary>
        private string CurrentEndpointKey()
        {
            if (_endpointKeyOverride != null) return _endpointKeyOverride;
            if (_connection == null)
                _connection = FindObjectOfType<ConnectionManager>();
            if (_connection == null) return "";
            return (_connection.CurrentIp ?? "") + ":" + _connection.CurrentPort;
        }

        private static bool SameEndpoint(string a, string b)
            => string.Equals(a ?? "", b ?? "", StringComparison.Ordinal);

        /// <summary>
        /// The scheme is latched for a previous connection. Clear it locally
        /// and do not publish emergency_stop false.
        /// </summary>
        private void ReleaseLatchIfEndpointChanged()
        {
            bool driveLatched = _drive is DriveSchemeBase drive && drive.IsEmergencyStopActive;
            if (!driveLatched && !_armEStopLatched) return;
            if (!_emergencyStopLatched || _emergencyStopEndpoint == null) return;
            if (SameEndpoint(_emergencyStopEndpoint, CurrentEndpointKey())) return;
            if (driveLatched && _drive is DriveSchemeBase scheme)
                scheme.DiscardEmergencyStopLatch();
            ClearArmEStopLocal();
            _emergencyStopLatched = false;
            _emergencyStopEndpoint = null;
            _emergencyStopPublishPending = false;
        }

        private void RememberLatchEndpoint()
        {
            bool driveLatched = _drive is DriveSchemeBase drive && drive.IsEmergencyStopActive;
            if (!driveLatched && !_armEStopLatched) return;
            if (_emergencyStopLatched && _emergencyStopEndpoint != null) return;
            _emergencyStopLatched = true;
            _emergencyStopEndpoint = CurrentEndpointKey();
        }

        private void TickArmOnlyEStop(ref TeleopInput input)
        {
            IRobotLink link = Link;
            bool linkUp = link != null && link.IsConnected;
            if (linkUp)
                FlushArmEStop(link);

            bool abHeld = input.ButtonA && input.ButtonB;
            bool chord = input.EmergencyStopPressed || input.HudEmergencyStop || input.BothGripsPressed
                || input.BothGrips || abHeld;
            if (chord)
            {
                _armEStopController = true;
                _armEStopResetAccepted = false;
                _armEStopDeadmanReleased = false;
                if (!_armEStopLatched)
                    ArmTriggerEStop(link);
                return;
            }

            // A dropped link cannot accept the clear, and it must not drop the latch.
            if (!linkUp) return;

            if (OperateSafetyLatch.UiEStopLatched)
            {
                _armEStopResetAccepted = false;
                _armEStopDeadmanReleased = false;
                if (!_armEStopLatched)
                    ArmTriggerEStop(link);
                return;
            }

            if (!_armEStopLatched || !_armEStopController) return;
            if (input.BothGrips || abHeld || (input.LeftGrip && input.RightGrip)) return;

            if (input.ResetEmergencyStop)
                _armEStopResetAccepted = true;
            bool deadman = input.LeftGrip;
            if (_armEStopResetAccepted && !deadman)
                _armEStopDeadmanReleased = true;
            if (_armEStopResetAccepted && _armEStopDeadmanReleased && deadman)
                ArmClearEStop(link);
        }

        private void ArmTriggerEStop(IRobotLink link)
        {
            _armEStopLatched = true;
            _armEStopResetAccepted = false;
            _armEStopDeadmanReleased = false;
            bool linkUp = link != null && link.IsConnected;
            if (linkUp)
                PublishArmEStop(link, true);
            _armEStopPublishPending = !linkUp;
            NavGoalSession.Active.SetHeadsetBlock(true, linkUp ? link : null);
        }

        private void ArmClearEStop(IRobotLink link)
        {
            _armEStopLatched = false;
            _armEStopController = false;
            _armEStopPublishPending = false;
            _armEStopResetAccepted = false;
            _armEStopDeadmanReleased = false;
            if (!OperateSafetyLatch.UiEStopLatched)
                NavGoalSession.Active.SetHeadsetBlock(false, null);
            PublishArmEStop(link, false);
        }

        private void RestoreArmEmergencyStop(bool publishPending)
        {
            _armEStopLatched = true;
            _armEStopController = true;
            _armEStopResetAccepted = false;
            _armEStopDeadmanReleased = false;
            _armEStopPublishPending = publishPending;
            NavGoalSession.Active.SetHeadsetBlock(true, null);
        }

        private void ClearArmEStopLocal()
        {
            _armEStopLatched = false;
            _armEStopController = false;
            _armEStopResetAccepted = false;
            _armEStopDeadmanReleased = false;
            _armEStopPublishPending = false;
        }

        private void FlushArmEStop(IRobotLink link)
        {
            if (!_armEStopPublishPending || !_armEStopLatched) return;
            if (link == null || !link.IsConnected) return;
            PublishArmEStop(link, true);
            _armEStopPublishPending = false;
        }

        private void PublishArmEStop(IRobotLink link, bool stop)
        {
            if (link == null || !link.IsConnected || Profile?.Topics == null) return;
            if (!string.IsNullOrEmpty(Profile.Topics.EmergencyStop))
                link.Publish(Profile.Topics.EmergencyStop, new BoolMsg(stop));
            if (stop && !string.IsNullOrEmpty(Profile.Topics.CmdVel))
                link.Publish(Profile.Topics.CmdVel, new TwistMsg(new Vector3Msg(0f, 0f, 0f), new Vector3Msg(0f, 0f, 0f)));
        }

        /// <summary>
        /// Takeoff button. The phrase has to match before anything is queued.
        /// Stick motion cannot call this.
        /// </summary>
        public bool RequestAerialTakeoff(string confirmation)
        {
            var aerial = _drive as AerialDriveScheme;
            if (aerial == null) return false;
            return aerial.TryTakeoff(confirmation);
        }

        /// <summary>Land button. Refuses until the confirmation phrase matches.</summary>
        public bool RequestAerialLand(string confirmation)
        {
            var aerial = _drive as AerialDriveScheme;
            if (aerial == null) return false;
            return aerial.TryLand(confirmation);
        }

        void AttachAerialClient(IDriveScheme drive)
        {
            var aerial = drive as AerialDriveScheme;
            if (aerial == null) return;
            if (_aerialClient == null)
                _aerialClient = GetComponent<SimServeAerialClient>();
            if (_aerialClient == null)
                _aerialClient = gameObject.AddComponent<SimServeAerialClient>();
            aerial.UseSender(_aerialClient);
            if (Application.isPlaying)
                _aerialClient.ConnectLoopback();
        }

        /// <summary>
        /// Use this transport for the next session. Null restores the ROSBridge link.
        /// The in-headset demo passes an <see cref="OfflineKinematicLink"/>, which
        /// never opens a socket to a real robot.
        /// </summary>
        public void SetLink(IRobotLink link)
        {
            _link = link ?? ResolveLink();
        }

        /// <summary>
        /// True when commands go out through <see cref="RosBridgeLink"/>.
        /// An assigned transport such as the play-mode fake link is false, so a
        /// session must not also open a rosbridge socket.
        /// </summary>
        public bool PublishesViaRosBridge => (_link ?? ResolveLink()) is RosBridgeLink;

        private void EnsureWorkspaceOrigin(RobotProfile profile)
        {
            if (handWorkspaceOrigin != null || profile == null) return;
            _ownedOrigin = new GameObject("HandWorkspaceOrigin");
            _ownedOrigin.transform.position = new Vector3(0f, profile.ArmBaseHeight, 0f);
            handWorkspaceOrigin = _ownedOrigin.transform;
            _originOwned = true;
        }

        private void EnsureWorkspaceVisual(RobotProfile profile)
        {
            if (handWorkspaceOrigin == null || profile == null) return;
            var go = new GameObject("ArmWorkspaceSphere");
            go.transform.SetParent(handWorkspaceOrigin, false);
            _workspaceVisual = go.AddComponent<WorkspaceSphereVisual>();
            _workspaceVisual.Configure(profile.HandWorkspaceRadius > 0f
                ? profile.HandWorkspaceRadius
                : RobotDescriptor.So101.HandWorkspaceRadius);
        }

        // ── Input helpers ─────────────────────────────────────────────────────────

        private HandPose BuildHandPose(OVRHand hand)
        {
            if (gestureDetector == null || hand == null || !gestureDetector.IsHandTracked(hand))
                return default;

            bool good = gestureDetector.IsHandTrackingGood(hand);
            return new HandPose
            {
                Tracked = true,
                Quality = good ? HandTrackingQuality.Good : HandTrackingQuality.Poor,
                Position = gestureDetector.GetHandPosition(hand),
                Rotation = gestureDetector.GetHandRotation(hand),
                PinchStrength = gestureDetector.GetPinchStrength(hand),
            };
        }

        // ── Thumbs-up arm-enable toggle (ported from HandTrackingArmController) ────
        private void TickThumbsUpToggle(OVRHand leftHand)
        {
            if (_manip == null || gestureDetector == null || leftHand == null ||
                !gestureDetector.IsHandTracked(leftHand)) return;

            bool thumbsUp = IsThumbsUp(leftHand);
            if (thumbsUp)
            {
                _thumbsUpHoldTimer += Time.deltaTime;
                if (!_wasThumbsUp && _thumbsUpHoldTimer >= ThumbsUpHoldRequired)
                {
                    // Request only. REAL confirmation is Left X, not this gesture.
                    _thumbsUpPulse = true;
                    _wasThumbsUp = true;
                }
            }
            else
            {
                _thumbsUpHoldTimer = 0f;
                _wasThumbsUp = false;
            }
        }

        private bool IsThumbsUp(OVRHand hand)
        {
            if (hand == null || gestureDetector == null || !gestureDetector.IsHandTracked(hand)) return false;

            OVRSkeleton skeleton = hand.GetComponent<OVRSkeleton>();
            if (skeleton == null || !skeleton.IsInitialized)
            {
                // Fallback: pinch-strength heuristic
                float indexPinch = hand.GetFingerPinchStrength(OVRHand.HandFinger.Index);
                float middlePinch = hand.GetFingerPinchStrength(OVRHand.HandFinger.Middle);
                float ringPinch = hand.GetFingerPinchStrength(OVRHand.HandFinger.Ring);
                float pinkyPinch = hand.GetFingerPinchStrength(OVRHand.HandFinger.Pinky);
                bool othersCurled = middlePinch > 0.6f && ringPinch > 0.6f && pinkyPinch > 0.6f;
                bool thumbFree = indexPinch < 0.3f;
                bool pointingUp = Vector3.Dot(hand.transform.up, Vector3.up) > 0.7f;
                return othersCurled && thumbFree && pointingUp;
            }

            var bones = skeleton.Bones;
            if (bones == null || bones.Count < 24) return false;

            Vector3 wristPos = bones[0].Transform.position;
            Vector3 thumbTip = bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
            Vector3 indexTip = bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
            Vector3 middleTip = bones[(int)OVRSkeleton.BoneId.Hand_MiddleTip].Transform.position;
            Vector3 ringTip = bones[(int)OVRSkeleton.BoneId.Hand_RingTip].Transform.position;
            Vector3 pinkyTip = bones[(int)OVRSkeleton.BoneId.Hand_PinkyTip].Transform.position;

            bool thumbExtended = thumbTip.y > wristPos.y + 0.04f;
            float curled = 0.07f;
            bool indexCurled = Vector3.Distance(indexTip, wristPos) < curled;
            bool middleCurled = Vector3.Distance(middleTip, wristPos) < curled;
            bool ringCurled = Vector3.Distance(ringTip, wristPos) < curled;
            bool pinkyCurled = Vector3.Distance(pinkyTip, wristPos) < curled;

            return thumbExtended && indexCurled && middleCurled && ringCurled && pinkyCurled;
        }

        // ── Topic advertising ─────────────────────────────────────────────────────
        private void AdvertiseTopics(RobotProfile p)
        {
            if (!Link.IsConnected || p?.Topics == null) return;

            if (!string.IsNullOrEmpty(p.Topics.CmdVel))
                Link.Advertise(p.Topics.CmdVel, "geometry_msgs/Twist");
            if (!string.IsNullOrEmpty(p.Topics.ControlMode))
                Link.Advertise(p.Topics.ControlMode, "std_msgs/String");
            if (!string.IsNullOrEmpty(p.Topics.ArmCommands))
                Link.Advertise(p.Topics.ArmCommands, "sensor_msgs/JointState");
            if (!string.IsNullOrEmpty(p.Topics.ArmEnable))
                Link.Advertise(p.Topics.ArmEnable, "std_msgs/Bool");
            if (!string.IsNullOrEmpty(p.Topics.EmergencyStop))
                Link.Advertise(p.Topics.EmergencyStop, "std_msgs/Bool");
            if (!string.IsNullOrEmpty(p.Topics.IkTargetPose))
                Link.Advertise(p.Topics.IkTargetPose, "geometry_msgs/PoseStamped");
            if (!string.IsNullOrEmpty(p.Topics.RecordStart))
                Link.Advertise(p.Topics.RecordStart, "std_msgs/String");
            if (!string.IsNullOrEmpty(p.Topics.RecordStop))
                Link.Advertise(p.Topics.RecordStop, "std_msgs/Bool");
        }
    }
}
