using UnityEngine;
using OmniBot.VR.Core;
using OmniBot.VR.Core.Platform;
using OmniBot.VR.Control;
using OmniBot.VR.Control.Session;
using OmniBot.VR.Policy;
using OmniBot.VR.UI;
using LatchedRobotDescriptor = OmniBot.VR.Descriptor.RobotDescriptor;
using OmniBot.VR.UI.Garage;
using OmniBot.VR.UI.Session;

namespace OmniBot.VR.App
{
    /// <summary>
    /// Top-level app flow for the OhhO VR headset — mirrors the website's own
    /// journey: <b>sign in → console (VR products) → open Pilot → garage → simulate or drive real → teleop</b>.
    ///
    /// It wires the platform services together once the manifest loads (so the
    /// Supabase auth + garage clients get the same project the website uses), then
    /// routes between panels based on auth state. A returning user with a saved
    /// session skips straight past login — the connected-experience goal: sign in
    /// once, anywhere, and your robots are already there.
    ///
    /// Drop this on a bootstrap GameObject alongside OhhoPlatform, OhhoCatalog,
    /// SupabaseAuthService and GarageClient, and assign the three panels.
    /// </summary>
    public class OhhoVrApp : MonoBehaviour
    {
        [Header("Panels (assign in the scene)")]
        [SerializeField] private GameObject loginPanel;
        [SerializeField] private GameObject consolePanel;
        [SerializeField] private GameObject garagePanel;
        [Tooltip("Simulate / Drive real, shown after a garage pick and before teleop.")]
        [SerializeField] private GameObject sessionPickerPanel;
        [Tooltip("The teleop page shown after a session target is chosen (camera feed, telemetry, recording).")]
        [SerializeField] private GameObject teleopPanel;
        [SerializeField] private GameObject cameraPanel;
        [SerializeField] private GameObject controlsPanel;

        [Header("Services")]
        [SerializeField] private OhhoPlatform platform;
        [SerializeField] private SupabaseAuthService auth;
        [SerializeField] private GarageClient garage;

        [Header("Teleop (Phase 2)")]
        [Tooltip("The garage panel controller — listened to for robot selection.")]
        [SerializeField] private GaragePanelController garagePanelController;
        [Tooltip("The fleet panel controller — enhanced garage with quick-resume. Optional.")]
        [SerializeField] private FleetPanelController fleetPanelController;
        [Tooltip("The teleop control layer. Receives the selected robot's profile.")]
        [SerializeField] private TeleopController teleopController;

        [Header("Phase 5 — polish")]
        [Tooltip("First-run onboarding overlay. Shows control hints on the first teleop session.")]
        [SerializeField] private OnboardingController onboarding;

        private SessionPickerPanel _picker;
        private bool _pickerHooked;
        private RobotProfile _pendingProfile;
        private string _pendingName;
        private string _pendingUserRobotId;
        private OfflineKinematicLink _offlineLink;
        private GarageRobot _activeRobot;
        private LatchedRobotDescriptor _descriptor;

        private void Awake()
        {
            if (platform == null) platform = OhhoPlatform.Instance;
            if (auth == null) auth = SupabaseAuthService.Instance;
            if (garage == null) garage = GarageClient.Instance;
        }

        private void OnEnable()
        {
            if (platform != null) platform.OnLoaded += OnManifestLoaded;
            if (auth != null) auth.OnAuthChanged += OnAuthChanged;
            if (garagePanelController != null) garagePanelController.RobotPicked += OnRobotPicked;
            if (fleetPanelController != null)
            {
                fleetPanelController.RobotPicked += OnRobotPicked;
                fleetPanelController.QuickResume += OnQuickResume;
            }

            var conn = ConnectionManager.Instance;
            if (conn != null)
            {
                conn.OnRobotDescriptor += OnRobotDescriptor;
                conn.OnConnectionStateChanged += OnLinkChanged;
                _descriptor = conn.LatestDescriptor;
            }
        }

        private void OnDisable()
        {
            if (platform != null) platform.OnLoaded -= OnManifestLoaded;
            if (auth != null) auth.OnAuthChanged -= OnAuthChanged;
            if (garagePanelController != null) garagePanelController.RobotPicked -= OnRobotPicked;
            if (fleetPanelController != null)
            {
                fleetPanelController.RobotPicked -= OnRobotPicked;
                fleetPanelController.QuickResume -= OnQuickResume;
            }
            UnhookPicker();

            var conn = ConnectionManager.Instance;
            if (conn != null)
            {
                conn.OnRobotDescriptor -= OnRobotDescriptor;
                conn.OnConnectionStateChanged -= OnLinkChanged;
            }
        }

        private void OnRobotDescriptor(LatchedRobotDescriptor descriptor)
        {
            _descriptor = descriptor;
            if (_activeRobot == null) return;
            if (teleopController != null && teleopController.IsTeleopActive)
            {
                ApplyActiveProfile(showUi: false);
                return;
            }

            // The picker is still open. Keep the profile that will be confirmed
            // on the descriptor so catalog speeds cannot outrank the robot.
            if (_pendingProfile != null)
            {
                var refreshed = BuildActiveProfile();
                if (refreshed != null) _pendingProfile = refreshed;
            }
        }

        private void OnLinkChanged(bool connected)
        {
            // Drop the cached document so the next pick does not reuse another robot's
            // topics. The profile already in teleop stays until a new document arrives,
            // so a disconnect cannot swap in a higher catalog speed.
            if (!connected) _descriptor = null;
        }

        private void Start()
        {
            // Show login immediately; the manifest load decides where we land.
            ShowLogin();
            if (platform != null && platform.Manifest != null) OnManifestLoaded(platform.Manifest);
        }

        private void OnManifestLoaded(VrManifest manifest)
        {
            // Configure auth + garage with the same Supabase project the site uses.
            auth?.Configure(manifest.Auth);
            garage?.Configure(manifest.Auth);

            // Returning user? Restore the session and skip login.
            if (auth != null)
                auth.RestoreSession(restored => { if (restored) ShowConsole(); else ShowLogin(); });
        }

        private async void OnAuthChanged(bool signedIn)
        {
            if (signedIn)
            {
                ShowConsole();
                return;
            }

            // Sign-out must stop the robot that is still linked. Hiding the
            // panels alone leaves teleop publishing.
            if (teleopController != null) teleopController.StopTeleop();
            if (ConnectionManager.HasInstance && ROSBridgeClient.HasInstance &&
                ConnectionManager.Instance.IsConnected)
                await ConnectionManager.Instance.DisconnectAsync();
            if (this == null) return;
            if (auth != null && auth.IsSignedIn) return;
            ShowLogin();
        }

        // ── Panel routing ─────────────────────────────────────────────────────
        public void ShowLogin() => Activate(login: true, console: false, garage: false, teleop: false);
        public void ShowConsole() => Activate(login: false, console: true, garage: false, teleop: false);
        public void ShowGarage() => Activate(login: false, console: false, garage: true, teleop: false);
        public void ShowSessionPicker()
        {
            EnsurePicker();
            Activate(login: false, console: false, garage: false, teleop: false, session: true);
        }
        public void ShowTeleop()
        {
            Activate(login: false, console: false, garage: false, teleop: true, camera: true, controls: true);
            if (ActiveSession.Current != null)
                PaintSessionBanners(ActiveSession.Current.Kind);
        }

        private void Activate(bool login, bool console, bool garage, bool teleop, bool camera = false, bool controls = false, bool session = false)
        {
            if (loginPanel != null) loginPanel.SetActive(login);
            if (consolePanel != null) consolePanel.SetActive(console);
            if (garagePanel != null) garagePanel.SetActive(garage);
            if (teleopPanel != null) teleopPanel.SetActive(teleop);
            if (cameraPanel != null) cameraPanel.SetActive(camera);
            if (controlsPanel != null) controlsPanel.SetActive(controls);
            if (sessionPickerPanel != null) sessionPickerPanel.SetActive(session);
        }

        /// <summary>Open ohho-robotics.com in the headset's system browser.</summary>
        public void OpenWebsite()
        {
            string url = platform != null ? platform.PlatformBaseUrl : "https://ohho-robotics.com";
            Application.OpenURL(url);
        }

        // ── Phase 2: robot selection → teleop ──────────────────────────────────

        /// <summary>
        /// Called when the user picks a robot in the garage. Builds the profile,
        /// then opens the session picker (Simulate / Drive real) instead of
        /// jumping straight to teleop.
        /// </summary>
        private void OnRobotPicked(GarageRobot robot)
        {
            BeginRobot(robot, quickResume: false);
        }

        /// <summary>
        /// Descriptor wins while connected. The catalog profile is the offline
        /// fallback and fills topics the descriptor does not name.
        /// </summary>
        private bool ApplyActiveProfile(bool showUi)
        {
            if (_activeRobot == null) return false;
            if (teleopController == null)
            {
                Debug.LogWarning("[OhhoVrApp] TeleopController not assigned — cannot start teleop.");
                return false;
            }

            var profile = BuildActiveProfile();
            if (profile == null)
            {
                Debug.LogWarning($"[OhhoVrApp] Could not build a profile for {_activeRobot.DisplayName}.");
                return false;
            }

            PreparePolicyPanel(ActiveSession.Current != null ? ActiveSession.Current.Kind : (SessionKind?)null);
            teleopController.StartTeleop(profile);
            if (showUi) ShowTeleop();
            return true;
        }

        /// <summary>
        /// Descriptor wins while connected. The catalog profile is the offline
        /// fallback and fills topics the descriptor does not name. Calibration
        /// cannot raise the sealed caps.
        /// </summary>
        private RobotProfile BuildActiveProfile()
        {
            if (_activeRobot == null) return null;
            var catalog = RobotProfileFactory.FromGarageRobot(_activeRobot);
            if (catalog == null) return null;

            var conn = ConnectionManager.Instance;
            if (_descriptor == null && conn != null)
                _descriptor = conn.LatestDescriptor;

            var profile = _descriptor != null
                ? RobotProfileFactory.FromDescriptor(_descriptor, catalog) ?? catalog
                : catalog;

            string calibrationId = string.IsNullOrEmpty(catalog.RobotId) ? profile.RobotId : catalog.RobotId;
            var cal = CalibrationManager.Load(calibrationId);
            CalibrationManager.ApplyTo(profile, cal);
            return profile;
        }

        /// <summary>
        /// Attach before StartTeleop so TeleopSessionStarted binds this profile.
        /// The session picker kind wins when the operator already chose sim or real.
        /// Otherwise the garage robot status is used.
        /// </summary>
        private void PreparePolicyPanel(SessionKind? sessionKind)
        {
            if (teleopController == null) return;
            var policyPanel = PolicyPanelController.FindOrCreate(teleopController);
            PolicySessionKind kind;
            if (sessionKind == SessionKind.Sim)
                kind = PolicySessionKind.Sim;
            else if (sessionKind == SessionKind.Real)
                kind = PolicySessionKind.Real;
            else
            {
                string robotStatus = _activeRobot != null && _activeRobot.UserRobot != null
                    ? _activeRobot.UserRobot.Status
                    : null;
                kind = PolicyPanelController.SessionKindForStatus(robotStatus);
            }
            policyPanel.PrepareSession(teleopController, kind);
        }

        /// <summary>
        /// Quick-resume: jump straight into teleop with the last-driven robot.
        /// Resolves the saved robot id from the catalog, builds + calibrates the
        /// profile, and starts teleop. Falls back to the garage if the robot is
        /// no longer in the user's account.
        /// </summary>
        private void OnQuickResume()
        {
            if (!LastRobotStore.HasLastRobot)
            {
                ShowGarage();
                return;
            }

            string robotId = LastRobotStore.LastRobotId;
            var catalog = OhhoCatalog.Instance;
            if (catalog == null || !catalog.IsLoaded) { ShowGarage(); return; }

            // Find the robot in the user's garage by hardware model id
            var garageClient = GarageClient.Instance;
            if (garageClient == null) { ShowGarage(); return; }

            garageClient.GetUserRobots((robots, err) =>
            {
                if (err != null || robots == null) { ShowGarage(); return; }
                Core.Platform.UserRobot match = null;
                foreach (var r in robots)
                {
                    if (r.HardwareModelId == robotId) { match = r; break; }
                }
                if (match == null) { ShowGarage(); return; }
                var resolved = catalog.Resolve(match);
                if (resolved == null) { ShowGarage(); return; }
                BeginRobot(resolved, quickResume: true);
            });
        }

        /// <summary>Stop teleoperation and return to the garage.</summary>
        public void StopTeleopAndReturnToGarage()
        {
            if (teleopController != null) teleopController.StopTeleop();
            ShowGarage();
        }

        // ── Session picker ────────────────────────────────────────────────────

        private void BeginRobot(GarageRobot robot, bool quickResume)
        {
            if (robot == null) return;
            _activeRobot = robot;
            if (teleopController == null)
            {
                Debug.LogWarning("[OhhoVrApp] TeleopController not assigned — cannot start teleop.");
                return;
            }

            var profile = BuildActiveProfile();
            if (profile == null)
            {
                Debug.LogWarning($"[OhhoVrApp] Could not build a profile for {robot.DisplayName}.");
                return;
            }

            _pendingProfile = profile;
            _pendingName = robot.DisplayName;
            _pendingUserRobotId = robot.UserRobot != null ? robot.UserRobot.Id : null;

            RobotEndpoint saved = null;
            bool hasChoice = !string.IsNullOrEmpty(_pendingUserRobotId) &&
                             LastRobotStore.TryLoadChoice(_pendingUserRobotId, out saved);
            if (!hasChoice)
                hasChoice = LastRobotStore.TryLoadChoice(profile.RobotId, out saved);

            // Quick-resume of a sim target skips the picker. A real robot still
            // stops on the safety notice — resume must not skip that.
            if (quickResume && hasChoice && saved != null && saved.Kind == SessionKind.Sim)
            {
                OnSessionConfirmed(SessionPlanner.FromSaved(saved, safetyAcknowledged: false, tutorialSeen: true));
                return;
            }

            // Simulate and Drive real keep separate addresses. A saved real host
            // must not prefill Simulate, and the last socket is not a sim target.
            string simIp = EndpointAddresses.DefaultIp;
            int simPort = EndpointAddresses.RosbridgePort;
            string realIp = EndpointAddresses.DefaultIp;
            int realPort = EndpointAddresses.RosbridgePort;
            if (hasChoice && saved != null && !string.IsNullOrEmpty(saved.Host))
            {
                int savedPort = saved.Port > 0 ? saved.Port : EndpointAddresses.RosbridgePort;
                if (saved.Kind == SessionKind.Real)
                {
                    realIp = saved.Host;
                    realPort = savedPort;
                }
                else
                {
                    simIp = saved.Host;
                    simPort = savedPort;
                }
            }

            var step = quickResume && hasChoice && saved != null && saved.Kind == SessionKind.Real
                ? PickerStep.RealSafety
                : PickerStep.Mode;
            EnsurePicker();
            ShowSessionPicker();
            _picker.Open(profile.RobotId, _pendingUserRobotId, robot.DisplayName,
                simIp, simPort, realIp, realPort, step);
        }

        private async void OnSessionConfirmed(SessionLaunch plan)
        {
            if (plan == null || !plan.ReadyToDrive || plan.Endpoint == null || _pendingProfile == null) return;
            if (teleopController == null) return;
            if (!plan.DeadmanRequired)
            {
                Debug.LogWarning("[OhhoVrApp] Refusing a session that would turn the deadman off.");
                return;
            }

            var endpoint = plan.Endpoint;
            if (endpoint.Kind == SessionKind.Real)
                endpoint.Offline = false;

            if (!endpoint.Offline && (endpoint.VideoUrls == null || endpoint.VideoUrls.Count == 0))
                endpoint.VideoUrls = EndpointAddresses.VideoUrlsFor(endpoint.Host, _pendingProfile.Topics != null
                    ? _pendingProfile.Topics.Images
                    : null);

            RealRobotSafety.Apply(_pendingProfile, endpoint.Kind);
            if (endpoint.Kind == SessionKind.Real && !_pendingProfile.RequireDeadman)
            {
                Debug.LogWarning("[OhhoVrApp] Refusing real teleop because the deadman is off.");
                return;
            }

            // Zero the robot on the link that is still installed, then wait until
            // that socket is closed before pointing teleop at the new target.
            teleopController.StopTeleop();
            if (ConnectionManager.HasInstance && ROSBridgeClient.HasInstance &&
                ConnectionManager.Instance.IsConnected)
                await ConnectionManager.Instance.DisconnectAsync();
            if (this == null || teleopController == null || _pendingProfile == null) return;

            ActiveSession.Set(endpoint);
            string host = endpoint.Offline ? "in-headset" : endpoint.Host;
            int port = endpoint.Port > 0 ? endpoint.Port : EndpointAddresses.RosbridgePort;
            LastRobotStore.Save(_pendingProfile.RobotId, _pendingName, host, port, endpoint);
            if (!string.IsNullOrEmpty(_pendingUserRobotId) && _pendingUserRobotId != _pendingProfile.RobotId)
            {
                LastRobotStore.SaveChoice(_pendingUserRobotId, endpoint);
                LastRobotStore.RememberAlias(_pendingUserRobotId);
            }

            if (endpoint.Offline)
            {
                if (_offlineLink == null) _offlineLink = new OfflineKinematicLink();
                _offlineLink.IsConnected = true;
                teleopController.SetLink(_offlineLink);
            }
            else
            {
                teleopController.SetLink(null);
                // The headset scene leaves linkProvider empty, so this is RosBridgeLink
                // and the endpoint is dialed. A stand-in transport already carries
                // teleop and must not also open a socket.
                if (teleopController.PublishesViaRosBridge)
                    ConnectionManager.Instance.ConnectEndpoint(endpoint);
            }

            PreparePolicyPanel(endpoint.Kind);
            teleopController.StartTeleop(_pendingProfile);
            ShowTeleop();
            PaintSessionBanners(endpoint.Kind);

            if (onboarding != null)
                onboarding.ShowIfFirstTime(teleopController.DriveHint, teleopController.ManipHint);
        }

        private void PaintSessionBanners(SessionKind kind)
        {
            SessionModeBanner.Apply(teleopPanel, kind);
            SessionModeBanner.Apply(cameraPanel, kind);
            SessionModeBanner.Apply(controlsPanel, kind);
            SessionModeBanner.Apply(garagePanel, kind);
            SessionModeBanner.Apply(SessionPage, kind);
        }

        private GameObject SessionPage => _picker != null ? _picker.gameObject : sessionPickerPanel;

        private void EnsurePicker()
        {
            if (_picker != null)
            {
                HookPicker();
                return;
            }

            if (sessionPickerPanel != null)
            {
                _picker = sessionPickerPanel.GetComponent<SessionPickerPanel>();
                if (_picker == null) _picker = sessionPickerPanel.AddComponent<SessionPickerPanel>();
            }
            else
            {
                var parent = garagePanel != null ? garagePanel.transform.parent : transform;
                var go = new GameObject("Session Page", typeof(RectTransform));
                go.transform.SetParent(parent, false);
                // Headset garage pages are UI rects. A plain object (the play-mode
                // harness) has no RectTransform; reading one throws MissingComponentException.
                if (garagePanel != null && garagePanel.TryGetComponent<RectTransform>(out var src))
                {
                    var dst = (RectTransform)go.transform;
                    dst.anchorMin = src.anchorMin;
                    dst.anchorMax = src.anchorMax;
                    dst.pivot = src.pivot;
                    dst.anchoredPosition = src.anchoredPosition;
                    dst.sizeDelta = src.sizeDelta;
                    dst.offsetMin = src.offsetMin;
                    dst.offsetMax = src.offsetMax;
                    dst.localRotation = src.localRotation;
                    dst.localScale = src.localScale;
                }
                go.SetActive(false);
                _picker = go.AddComponent<SessionPickerPanel>();
                sessionPickerPanel = go;
            }

            HookPicker();
        }

        private void HookPicker()
        {
            if (_pickerHooked || _picker == null) return;
            _picker.Confirmed += OnSessionConfirmed;
            _picker.Cancelled += ShowGarage;
            _pickerHooked = true;
        }

        private void UnhookPicker()
        {
            if (!_pickerHooked || _picker == null) return;
            _picker.Confirmed -= OnSessionConfirmed;
            _picker.Cancelled -= ShowGarage;
            _pickerHooked = false;
        }
    }
}
