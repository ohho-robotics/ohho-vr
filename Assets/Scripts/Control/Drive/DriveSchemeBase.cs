using UnityEngine;
using OmniBot.VR.Core;
using OmniBot.VR.Nav;

namespace OmniBot.VR.Control.Drive
{
    /// <summary>
    /// Shared infrastructure for every <see cref="IDriveScheme"/> — the common
    /// plumbing that doesn't change between a mecanum base, a drone, or an
    /// underwater ROV: latched emergency stop (A+B, the HUD button, both grips,
    /// or the operate-screen latch; a controller latch clears only after Reset,
    /// a deadman release, and a re-press, and the operate latch blocks that
    /// clear until the operate bar resets it),
    /// turbo (right trigger, sim sessions only, then clamped to the robot-class
    /// hard cap), deadman (left grip, on unless this is a kind=sim session that
    /// opted out), thumbstick dead-zone, 20 Hz publish-rate timing, zero-stop
    /// after 0.1 s idle, and the <c>/control_mode</c> teleop↔idle management.
    /// Real robots (<c>kind=real</c>) do not publish <c>nav2</c> on idle — that
    /// mode is the Nav2 button or a new goal. Any stick input while Nav2 is
    /// active cancels the goal and returns to teleop. A latched e-stop cancels
    /// a Nav2 goal and refuses another until that latch is cleared.
    ///
    /// Subclasses implement <see cref="ComputeVelocity"/> — the scheme-specific
    /// mapping of dead-zoned thumbsticks + turbo flag → a
    /// <see cref="TwistMsg"/>. Everything else is handled here.
    /// </summary>
    public abstract class DriveSchemeBase : IDriveScheme
    {
        protected const float DeadZone = 0.1f;
        protected const float TurboMultiplier = 2.0f;

        /// <summary>
        /// OmniBot hard linear cap, in m/s. Same constant as
        /// <see cref="RobotConfig.MAX_LINEAR_VEL"/>. Per-robot ceilings live on
        /// <see cref="RobotProfile.HardMaxLinVel"/>; this remains the OmniBot value
        /// callers and tests compare against.
        /// </summary>
        public const float HardTurboLinCapMps = RobotConfig.MAX_LINEAR_VEL;

        protected const float ZeroStopDelay = 0.1f;       // s before we stop publishing zeros
        protected const float IdleModeRevertDelay = 0.5f; // s idle → revert control mode (sim / non-nav2)
        protected const float PublishHz = 20f;

        protected RobotProfile Profile;
        protected float PublishInterval;
        protected float PublishTimer;

        // ── E-stop / mode state ──────────────────────────────────────────────────
        protected bool EmergencyStopActive;
        protected bool IsPublishing;
        protected float NoInputTimer;
        private readonly ControlModeMachine _mode = new ControlModeMachine();
        private bool _publishEStopClear;
        private float _prevVx, _prevVy, _prevVz, _prevWz;

        // Operator clear is two more steps after the latch: an explicit Reset,
        // then the scheme deadman must be released and pressed again. Ground
        // schemes use the left grip. Aerial uses either grip, and never both.
        private bool _resetAccepted;
        private bool _deadmanReleasedAfterReset;

        /// <summary>
        /// Set when A+B, the HUD button, or both grips latched this stop.
        /// Operate reset clears only the operate-screen hold. This latch stays
        /// until Reset, a deadman release, and a re-press.
        /// </summary>
        private bool _controllerStopLatched;

        /// <summary>
        /// True when A+B, the HUD button, or both grips own the stop.
        /// An operate-bar stop leaves this false so operate Reset can clear it.
        /// </summary>
        public bool ControllerStopLatched => _controllerStopLatched;

        /// <summary>
        /// Latched while the link was down, so the next connected tick must
        /// publish emergency_stop true. A clear while disconnected is ignored.
        /// </summary>
        private bool _estopPublishPending;

        public bool IsConfigured => Profile != null;
        public abstract string HudHint { get; }

        /// <summary>
        /// True while A+B, the HUD button, both grips, or the operate-screen latch is holding the base.
        /// </summary>
        public bool IsEmergencyStopActive => EmergencyStopActive;

        /// <summary>
        /// True when the latch was taken while the link could not accept
        /// <c>emergency_stop</c> true. The next connected tick publishes it.
        /// </summary>
        public bool EmergencyStopPublishPending => _estopPublishPending;

        /// <summary>
        /// When true, the left grip must be held to drive. Releasing it publishes
        /// zero velocity on that same tick. Defaults to true. Assigning false is
        /// ignored unless the configured profile is a kind=sim session; call
        /// <see cref="DisableDeadmanForSimulation"/> to opt out explicitly.
        /// </summary>
        public bool RequireDeadman
        {
            get => _requireDeadman;
            set
            {
                if (!value && (Profile == null || Profile.Kind != Session.SessionKind.Sim || !RobotProfile.DeadmanOptOutAllowed))
                {
                    _requireDeadman = true;
                    return;
                }
                _requireDeadman = value;
            }
        }

        private bool _requireDeadman = true;

        /// <summary>Linear speed scale (m/s) for the tick in progress, already under the hard cap.</summary>
        protected float ActiveLinVelLimit;

        /// <summary>Angular speed scale (rad/s) for the tick in progress, already under the hard cap.</summary>
        protected float ActiveAngVelLimit;

        /// <summary>
        /// While true, thumbstick velocity is not published. E-stop edges still
        /// run. <see cref="Safety.TeleopSafetySession"/> sets this when the robot
        /// e-stop is latched, or until a deadman re-press after link loss or the
        /// headset coming off. Cleared on <see cref="Configure"/>.
        /// </summary>
        public bool SuppressMotion { get; set; }

        /// <summary>
        /// HUD fragment shared by every scheme. Always states whether the deadman
        /// is on, whether turbo is available, and that e-stop is latched.
        /// </summary>
        protected string SafetyHudSuffix
        {
            get
            {
                string deadman = RequireDeadman
                    ? " · Deadman ON (left grip required)"
                    : " · Deadman OFF (sim session)";
                string turbo = Profile != null && Profile.Kind == Session.SessionKind.Sim
                    ? $" · Right trigger: turbo (clamped to {LinearCapLabel()} m/s)"
                    : " · Turbo disabled (real robot)";
                return deadman + turbo
                    + " · A+B, HUD, or both grips: e-stop (latched until Reset, then release and re-press the deadman)";
            }
        }

        /// <summary>Control-mode string published after the idle timeout.</summary>
        protected virtual string IdleMode => "nav2";

        /// <summary>Control-mode string published while the user is driving.</summary>
        protected virtual string ActiveMode => "teleop";

        public virtual void Configure(RobotProfile profile)
        {
            Profile = profile;
            PublishInterval = 1f / PublishHz;
            PublishTimer = 0f;
            // Do not clear EmergencyStopActive. Reconfigure must not drop a latch
            // the robot still holds. A replacement scheme starts clear; TeleopController
            // copies the latch onto it and only an explicit reset publishes false.
            // The operator clear cycle starts over so a half-finished reset cannot
            // complete against a scheme that was just rebound.
            _resetAccepted = false;
            _deadmanReleasedAfterReset = false;
            IsPublishing = false;
            NoInputTimer = 0f;
            _mode.IdleDelay = IdleModeRevertDelay;
            _mode.Reset(IdleMode, profile != null ? profile.DeploymentKind : RobotDeployment.Real);
            _mode.ActiveMode = ActiveMode;
            if (profile != null)
            {
                RobotClassCaps.Ensure(profile);
                NavGoalSession.Active.BeginRobot(profile);
            }
            // Reconfigure keeps a latch. The goal that belonged to this robot
            // must not keep running underneath it.
            if (EmergencyStopActive || OperateSafetyLatch.UiEStopLatched)
                NavGoalSession.Active.SetHeadsetBlock(true, null);
            _publishEStopClear = false;
            // Null profile keeps the safe default (deadman on). False sticks only
            // when this profile is a kind=sim session.
            RequireDeadman = profile == null || profile.RequireDeadman;
            ActiveLinVelLimit = profile != null ? Mathf.Max(0f, profile.MaxLinVel) : 0f;
            ActiveAngVelLimit = profile != null ? Mathf.Max(0f, profile.MaxAngVel) : 0f;
            SuppressMotion = false;
            _prevVx = _prevVy = _prevVz = _prevWz = 0f;
        }

        /// <summary>
        /// Clear an operate-screen stop after <see cref="OperateSafetyLatch"/> is cleared.
        /// Refuses while that latch is still set, while a grip is held, and while
        /// A+B, the HUD button, or both grips own the stop. Those stay latched
        /// until Reset, a deadman release, and a re-press. This method does not
        /// publish <c>emergency_stop</c> false for that controller stop, including
        /// a stop that was taken while the link was down.
        /// </summary>
        public void ReleaseUiStop(IRobotLink link, bool gripHeld)
        {
            if (OperateSafetyLatch.UiEStopLatched || gripHeld || _controllerStopLatched) return;
            EmergencyStopActive = false;
            _estopPublishPending = false;
            _resetAccepted = false;
            _deadmanReleasedAfterReset = false;
            _publishEStopClear = true;
            NavGoalSession.Active.SetHeadsetBlock(false, null);
            PublishEStopClear(link);
        }

        /// <summary>
        /// Simulation-only deadman opt-out. No effect unless the profile's session
        /// kind is <see cref="Session.SessionKind.Sim"/>. Device builds keep the deadman required.
        /// </summary>
        public void DisableDeadmanForSimulation()
        {
            if (Profile == null || Profile.Kind != Session.SessionKind.Sim || !RobotProfile.DeadmanOptOutAllowed) return;
            Profile.DisableDeadmanForSimulation();
            _requireDeadman = false;
        }

        /// <summary>True when the left grip (the deadman) is held.</summary>
        public virtual bool IsDeadmanEngaged(in TeleopInput input)
        {
            return input.LeftGrip;
        }

        public virtual void Tick(ref TeleopInput input, IRobotLink link)
        {
            if (!IsConfigured || Profile == null) return;

            // Sample the latch before any publish. A one-frame A+B edge during
            // a drop must stick, and a clear while the link is down would drop
            // the local latch without the robot seeing emergency_stop false.
            if (link == null || !link.IsConnected)
            {
                LatchEmergencyStopWhileLinkDown(ref input, link);
                return;
            }

            FlushPendingEmergencyStop(link);

            RobotClassCaps.Ensure(Profile);
            if (_publishEStopClear && !OperateSafetyLatch.UiEStopLatched)
                PublishEStopClear(link);

            // Emergency stop: A+B, HUD, both grips, or the operate latch.
            // A controller latch clears only after Reset plus a deadman release
            // and re-press. The operate latch refuses that clear.
            HandleEmergencyStop(ref input, link);
            if (HoldUiEStop(link)) return;
            if (EmergencyStopActive)
            {
                if (IsPublishing && !string.IsNullOrEmpty(Profile.Topics?.CmdVel))
                {
                    link.Publish(Profile.Topics.CmdVel, ZeroTwist());
                    IsPublishing = false;
                }
                // Idle must not fall through to nav2 while the latch is held.
                LatchNavGoal(link);
                _mode.Stop();
                return;
            }

            // ── Deadman: left grip. Release publishes zero on this same tick. ──
            bool deadmanEngaged = !RequireDeadman || IsDeadmanEngaged(input);
            if (!deadmanEngaged)
            {
                // Deadman released: immediately halt base motion and revert control mode
                if (IsPublishing && !string.IsNullOrEmpty(Profile.Topics?.CmdVel))
                {
                    link.Publish(Profile.Topics.CmdVel, ZeroTwist());
                    IsPublishing = false;
                }
                UpdateControlMode(false, input.DeltaTime, link);
                return;
            }

            // Latched safety state, or motion held until a deadman re-press.
            if (SuppressMotion)
            {
                if (IsPublishing && !string.IsNullOrEmpty(Profile.Topics?.CmdVel))
                {
                    link.Publish(Profile.Topics.CmdVel, ZeroTwist());
                    IsPublishing = false;
                }
                UpdateControlMode(false, input.DeltaTime, link);
                return;
            }

            // ── Thumbsticks (dead-zoned) ───────────────────────────────────────
            float lx = ApplyDeadZone(input.LeftStick.x);
            float ly = ApplyDeadZone(input.LeftStick.y);
            float rx = ApplyDeadZone(input.RightStick.x);
            float ry = ApplyDeadZone(input.RightStick.y);

            // Turbo is the right trigger, and only in a kind=sim session. Real
            // robots ignore it. The multiplier is applied by the scheme; the hard
            // cap below is applied after that so turbo cannot exceed the class ceiling.
            bool turbo = Profile.Kind == Session.SessionKind.Sim && input.RightTrigger;
            float speedMult = turbo ? TurboMultiplier : 1.0f;

            float hardLin = Profile.HardMaxLinVel > 0f ? Profile.HardMaxLinVel : HardTurboLinCapMps;
            float hardAng = Profile.HardMaxAngVel > 0f ? Profile.HardMaxAngVel : RobotConfig.MAX_ANGULAR_VEL;
            float calibratedLin = Mathf.Min(Mathf.Max(0f, Profile.MaxLinVel), hardLin);
            float calibratedAng = Mathf.Min(Mathf.Max(0f, Profile.MaxAngVel), hardAng);
            ActiveLinVelLimit = calibratedLin;
            ActiveAngVelLimit = calibratedAng;

            // ── Scheme-specific velocity ───────────────────────────────────────
            bool hasInput;
            TwistMsg twist = ComputeVelocity(lx, ly, rx, ry, speedMult, out hasInput);
            // Slew and per-axis ceilings first, then the active linear limit.
            // LimitTwist caps each axis, so a holonomic or thruster diagonal
            // can still exceed a calibration-lowered MaxLinVel. The magnitude
            // clamp uses that active limit, which is already at or under the
            // class hard cap. Turbo was applied inside ComputeVelocity.
            SafetyCaps.LimitTwist(twist, Profile, ref _prevVx, ref _prevVy, ref _prevVz, ref _prevWz, input.DeltaTime);
            ClampLinearSpeed(twist, ActiveLinVelLimit);
            ClampAngularSpeed(twist, hardAng);
            if (twist != null && twist.linear != null)
            {
                _prevVx = twist.linear.x;
                _prevVy = twist.linear.y;
                _prevVz = twist.linear.z;
            }
            if (twist != null && twist.angular != null)
                _prevWz = twist.angular.z;

            // ── Control-mode management ────────────────────────────────────────
            UpdateControlMode(hasInput, input.DeltaTime, link);

            // ── Publish at 20 Hz ───────────────────────────────────────────────
            PublishTimer += input.DeltaTime;
            if (PublishTimer < PublishInterval) return;
            PublishTimer -= PublishInterval;

            bool shouldPublish = hasInput || NoInputTimer < ZeroStopDelay;
            if (string.IsNullOrEmpty(Profile.Topics.CmdVel)) return;
            if (shouldPublish)
            {
                link.Publish(Profile.Topics.CmdVel, twist);
                IsPublishing = true;
            }
            else if (IsPublishing)
            {
                PublishZero(link);
                IsPublishing = false;
            }
        }

        public virtual void Stop(IRobotLink link)
        {
            // Headset haptics stop with teleop. The latch stays set, and this
            // does not publish emergency_stop false — only ClearEmergencyStop
            // does that, on an explicit reset.
            VibrateControllers(0f);
            if (!IsConfigured || Profile == null) return;
            bool estopHold = EmergencyStopActive
                || OperateSafetyLatch.UiEStopLatched
                || NavGoalSession.Active.CommandsBlocked;
            bool wasNav = NavGoalSession.Active.GoalActive || NavGoalSession.Active.ModeIsNav2;
            bool cancelNav = !estopHold
                && RobotDeployment.IsReal(Profile.DeploymentKind)
                && wasNav;
            if (estopHold)
            {
                if (EmergencyStopActive || OperateSafetyLatch.UiEStopLatched)
                    NavGoalSession.Active.SetHeadsetBlock(true, link);
                else
                    NavGoalSession.Active.CancelForEStop(link);
            }
            else if (cancelNav)
            {
                NavGoalSession.Active.CancelFromButton(link);
            }
            if (link != null && link.IsConnected)
            {
                PublishZero(link);
                if (estopHold)
                {
                    _mode.Stop();
                    if (wasNav)
                    {
                        PublishControlMode(ActiveMode, link);
                        NavGoalSession.Active.NoteModePublished(ActiveMode);
                    }
                }
                else
                {
                    string idle = _mode.Stop();
                    if (cancelNav)
                    {
                        // Leave the real robot in teleop so a late Nav2 command cannot run.
                        PublishControlMode(ActiveMode, link);
                        NavGoalSession.Active.NoteModePublished(ActiveMode);
                    }
                    else if (!string.IsNullOrEmpty(idle))
                    {
                        PublishControlMode(idle, link);
                        NavGoalSession.Active.NoteModePublished(idle);
                    }
                }
            }
            else
            {
                _mode.Stop();
            }
            IsPublishing = false;
            _prevVx = _prevVy = _prevVz = _prevWz = 0f;
            NoInputTimer = 0f;
        }

        /// <summary>
        /// Re-apply a latch from the scheme this one replaces.
        /// <paramref name="publishPending"/> is true when that stop never reached
        /// the robot. The next connected tick publishes <c>emergency_stop</c> true.
        /// A stop the robot already holds is restored with
        /// <paramref name="publishPending"/> false and does not publish.
        /// <paramref name="controllerOwned"/> keeps the original source: true for
        /// A+B, the HUD button, or both grips, and false for an operate-only stop
        /// so operate Reset can still clear that restore.
        /// </summary>
        public void RestoreEmergencyStopLatch(bool publishPending = false, bool controllerOwned = true)
        {
            EmergencyStopActive = true;
            _controllerStopLatched = controllerOwned;
            IsPublishing = false;
            _estopPublishPending = publishPending;
            // No link: a restore must not publish. The next connected tick
            // flushes emergency_stop and cancels a goal that is still active.
            NavGoalSession.Active.SetHeadsetBlock(true, null);
            _mode.Stop();
        }

        /// <summary>
        /// Operate-bar latch. Cancels Nav2 immediately, before the next drive tick.
        /// </summary>
        public void HoldNavForLatchedStop(IRobotLink link)
        {
            LatchNavGoal(link);
            _mode.Stop();
        }

        /// <summary>
        /// Robot <c>/ohho/safety/state</c> latch. Cancels Nav2 while it is held
        /// and does not clear a headset latch when the robot latch drops.
        /// </summary>
        public void NoteRobotEStop(bool latched, IRobotLink link)
        {
            bool wasNav = latched && (NavGoalSession.Active.GoalActive || NavGoalSession.Active.ModeIsNav2);
            NavGoalSession.Active.SetRobotBlock(latched, link);
            if (!latched) return;
            _mode.Stop();
            if (!wasNav) return;
            PublishControlMode(ActiveMode, link);
            NavGoalSession.Active.NoteModePublished(ActiveMode);
        }

        /// <summary>
        /// Drop the local latch without publishing. Used when the active
        /// connection is no longer the endpoint that latched, so this scheme
        /// must not stay blocked and must not send emergency_stop false.
        /// </summary>
        public void DiscardEmergencyStopLatch()
        {
            EmergencyStopActive = false;
            _controllerStopLatched = false;
            _estopPublishPending = false;
            IsPublishing = false;
            NavGoalSession.Active.SetHeadsetBlock(false, null);
            VibrateControllers(0f);
        }

        // ── To override ──────────────────────────────────────────────────────────

        /// <summary>
        /// Map dead-zoned thumbstick values + speed multiplier → a Twist message.
        /// Scale linear axes by <see cref="ActiveLinVelLimit"/> and angular axes by
        /// <see cref="ActiveAngVelLimit"/>. Both are already at or under the class
        /// hard cap; <see cref="Tick"/> clamps linear magnitude to
        /// <see cref="ActiveLinVelLimit"/> again after this returns.
        /// <paramref name="speedMult"/> is 1.0, or 2.0 when sim turbo is held.
        /// </summary>
        protected abstract TwistMsg ComputeVelocity(
            float lx, float ly, float rx, float ry, float speedMult, out bool hasInput);

        // ── Shared helpers ────────────────────────────────────────────────────────

        public void TriggerEmergencyStop(IRobotLink link)
        {
            EmergencyStopActive = true;
            _resetAccepted = false;
            _deadmanReleasedAfterReset = false;
            LatchNavGoal(link);
            _mode.Stop();
            bool linkUp = link != null && link.IsConnected;
            if (linkUp && Profile?.Topics != null)
            {
                if (!string.IsNullOrEmpty(Profile.Topics.EmergencyStop))
                    link.Publish(Profile.Topics.EmergencyStop, new BoolMsg(true));
                if (!string.IsNullOrEmpty(Profile.Topics.CmdVel))
                    link.Publish(Profile.Topics.CmdVel, ZeroTwist());
            }
            // A drop cannot accept the publish. Hold it until the link returns.
            _estopPublishPending = !linkUp;
            IsPublishing = false;
            VibrateControllers(1f);
        }

        /// <summary>
        /// System clear (session teardown, tests). The operator path does not call
        /// this: releasing the buttons does nothing, and a Reset only arms the
        /// deadman release/re-press cycle inside <see cref="HandleEmergencyStop"/>.
        /// </summary>
        public void ClearEmergencyStop(IRobotLink link)
        {
            EmergencyStopActive = false;
            _controllerStopLatched = false;
            _estopPublishPending = false;
            _resetAccepted = false;
            _deadmanReleasedAfterReset = false;
            if (!OperateSafetyLatch.UiEStopLatched)
                NavGoalSession.Active.SetHeadsetBlock(false, null);
            if (link != null && link.IsConnected && Profile?.Topics != null)
            {
                if (!string.IsNullOrEmpty(Profile.Topics.EmergencyStop))
                    link.Publish(Profile.Topics.EmergencyStop, new BoolMsg(false));
            }
            VibrateControllers(0f);
        }

        /// <summary>
        /// Latch on A+B, the HUD button, or both grips. Clear only after Reset,
        /// a deadman release, and a re-press. Ground's deadman is the left grip.
        /// Aerial's is either grip. Holding either chord blocks the clear.
        /// A level both-grips or A+B hold latches even without the edge flag,
        /// so a chord still held after a dropped edge latches on the next tick.
        /// </summary>
        protected void HandleEmergencyStop(ref TeleopInput input, IRobotLink link)
        {
            bool abHeld = input.ButtonA && input.ButtonB;
            bool latch = input.EmergencyStopPressed || input.HudEmergencyStop || input.BothGripsPressed
                || (input.BothGrips && !EmergencyStopActive)
                || (abHeld && !EmergencyStopActive);
            if (latch)
            {
                _controllerStopLatched = true;
                _resetAccepted = false;
                _deadmanReleasedAfterReset = false;
                if (!EmergencyStopActive)
                    TriggerEmergencyStop(link);
                else
                    Debug.Log($"[{GetType().Name}] Emergency stop still latched.");
                return;
            }
            // The operate-screen latch is its own hold. Left X cannot clear it,
            // and it blocks the controller clear until the operate bar resets it.
            if (OperateSafetyLatch.UiEStopLatched)
            {
                _resetAccepted = false;
                _deadmanReleasedAfterReset = false;
                if (!EmergencyStopActive)
                    TriggerEmergencyStop(link);
                return;
            }

            if (!EmergencyStopActive) return;

            // Holding either e-stop chord blocks the clear. Both physical grips
            // count even when the BothGrips flag was not set on this sample.
            if (input.BothGrips || abHeld || (input.LeftGrip && input.RightGrip)) return;

            if (input.ResetEmergencyStop)
                _resetAccepted = true;

            // The release has to be observed after Reset. A grip that was already
            // up on the Reset tick counts; the re-press is a later tick.
            bool deadman = IsResetDeadmanHeld(input);
            if (_resetAccepted && !deadman)
                _deadmanReleasedAfterReset = true;

            if (_resetAccepted && _deadmanReleasedAfterReset && deadman)
            {
                ClearEmergencyStop(link);
                Debug.Log($"[{GetType().Name}] Emergency stop reset after deadman re-press.");
            }
        }

        /// <summary>
        /// Grip that completes e-stop clear. Ground stays the left grip.
        /// Aerial overrides this so either grip matches its deadman.
        /// </summary>
        protected virtual bool IsResetDeadmanHeld(in TeleopInput input)
        {
            return input.LeftGrip;
        }

        /// <summary>
        /// Latch A+B, the HUD button, or both grips while nothing can be published.
        /// Does not run the clear sequence. The stop stays a controller latch, so
        /// operate reset cannot publish <c>emergency_stop</c> false for it.
        /// </summary>
        protected void LatchEmergencyStopWhileLinkDown(ref TeleopInput input, IRobotLink link)
        {
            bool chord = input.EmergencyStopPressed || input.HudEmergencyStop || input.BothGripsPressed
                || input.BothGrips || (input.ButtonA && input.ButtonB);
            if (!chord) return;
            _controllerStopLatched = true;
            if (!EmergencyStopActive)
                TriggerEmergencyStop(link);
            else
            {
                _resetAccepted = false;
                _deadmanReleasedAfterReset = false;
            }
        }

        /// <summary>
        /// Publish a stop that was latched during a drop. Does not restart the
        /// clear cycle, so a Reset already in progress is left alone.
        /// </summary>
        protected void FlushPendingEmergencyStop(IRobotLink link)
        {
            if (!_estopPublishPending || !EmergencyStopActive) return;
            if (link == null || !link.IsConnected) return;
            if (Profile?.Topics != null)
            {
                if (!string.IsNullOrEmpty(Profile.Topics.EmergencyStop))
                    link.Publish(Profile.Topics.EmergencyStop, new BoolMsg(true));
                if (!string.IsNullOrEmpty(Profile.Topics.CmdVel))
                    link.Publish(Profile.Topics.CmdVel, ZeroTwist());
            }
            IsPublishing = false;
            _estopPublishPending = false;
        }

        private string LinearCapLabel()
        {
            float cap = Profile != null && Profile.HardMaxLinVel > 0f
                ? Profile.HardMaxLinVel
                : HardTurboLinCapMps;
            return cap.ToString("0.##", System.Globalization.CultureInfo.InvariantCulture);
        }

        private static void VibrateControllers(float amplitude)
        {
            try
            {
                if (Application.isPlaying)
                {
                    OVRInput.SetControllerVibration(amplitude, amplitude, OVRInput.Controller.LTouch);
                    OVRInput.SetControllerVibration(amplitude, amplitude, OVRInput.Controller.RTouch);
                }
            }
            catch
            {
                // Ignored in test runner / headless mode where OVRPlugin is not loaded
            }
        }

        /// <summary>
        /// Cancel a live Nav2 goal and hold the headset block. Publishes teleop
        /// once when the robot was in nav2, and nothing when it was not.
        /// </summary>
        private void LatchNavGoal(IRobotLink link)
        {
            bool wasNav = NavGoalSession.Active.GoalActive || NavGoalSession.Active.ModeIsNav2;
            NavGoalSession.Active.SetHeadsetBlock(true, link);
            if (!wasNav) return;
            PublishControlMode(ActiveMode, link);
            NavGoalSession.Active.NoteModePublished(ActiveMode);
        }

        private void UpdateControlMode(bool hasInput, float dt, IRobotLink link)
        {
            if (hasInput) NoInputTimer = 0f;
            else NoInputTimer += dt;

            _mode.ActiveMode = ActiveMode;
            NavStickHandover.Apply(
                _mode,
                NavGoalSession.Active,
                link,
                Profile.Topics.ControlMode,
                hasInput,
                dt);
        }

        protected void PublishControlMode(string mode, IRobotLink link)
        {
            if (link == null || string.IsNullOrEmpty(Profile?.Topics?.ControlMode) || string.IsNullOrEmpty(mode))
                return;
            link.Publish(Profile.Topics.ControlMode, new StringMsg(mode));
        }

        private void PublishZero(IRobotLink link)
        {
            if (link == null || string.IsNullOrEmpty(Profile?.Topics?.CmdVel)) return;
            link.Publish(Profile.Topics.CmdVel, ZeroTwist());
            _prevVx = _prevVy = _prevVz = _prevWz = 0f;
        }

        private bool HoldUiEStop(IRobotLink link)
        {
            if (!OperateSafetyLatch.UiEStopLatched) return false;
            LatchNavGoal(link);
            _mode.Stop();
            if (!EmergencyStopActive)
            {
                EmergencyStopActive = true;
                if (Profile.Topics != null && !string.IsNullOrEmpty(Profile.Topics.EmergencyStop))
                    link.Publish(Profile.Topics.EmergencyStop, new BoolMsg(true));
                if (Profile.Topics != null && !string.IsNullOrEmpty(Profile.Topics.CmdVel))
                    link.Publish(Profile.Topics.CmdVel, ZeroTwist());
            }
            if (IsPublishing && Profile.Topics != null && !string.IsNullOrEmpty(Profile.Topics.CmdVel))
            {
                link.Publish(Profile.Topics.CmdVel, ZeroTwist());
                IsPublishing = false;
            }
            return true;
        }

        private void PublishEStopClear(IRobotLink link)
        {
            if (!_publishEStopClear || OperateSafetyLatch.UiEStopLatched) return;
            if (link == null || !link.IsConnected || Profile?.Topics == null) return;
            if (!string.IsNullOrEmpty(Profile.Topics.EmergencyStop))
                link.Publish(Profile.Topics.EmergencyStop, new BoolMsg(false));
            if (!string.IsNullOrEmpty(Profile.Topics.CmdVel))
                link.Publish(Profile.Topics.CmdVel, ZeroTwist());
            _publishEStopClear = false;
        }

        protected static float ApplyDeadZone(float value)
        {
            if (Mathf.Abs(value) < DeadZone) return 0f;
            float sign = Mathf.Sign(value);
            float scaled = (Mathf.Abs(value) - DeadZone) / (1f - DeadZone);
            return sign * Mathf.Clamp01(scaled);
        }

        protected static TwistMsg ZeroTwist()
            => new TwistMsg(new Vector3Msg(0f, 0f, 0f), new Vector3Msg(0f, 0f, 0f));

        /// <summary>Clamp the linear vector so its magnitude cannot exceed <paramref name="limit"/> m/s.</summary>
        protected static void ClampLinearSpeed(TwistMsg twist, float limit)
        {
            if (twist?.linear == null) return;
            float cap = Mathf.Abs(limit);
            var v = new Vector3(twist.linear.x, twist.linear.y, twist.linear.z);
            if (v.sqrMagnitude <= cap * cap) return;
            v = v.normalized * cap;
            twist.linear.x = v.x;
            twist.linear.y = v.y;
            twist.linear.z = v.z;
        }

        /// <summary>Clamp the angular vector so its components cannot exceed <paramref name="limit"/> rad/s.</summary>
        protected static void ClampAngularSpeed(TwistMsg twist, float limit)
        {
            if (twist?.angular == null) return;
            float cap = Mathf.Abs(limit);
            twist.angular.x = Mathf.Clamp(twist.angular.x, -cap, cap);
            twist.angular.y = Mathf.Clamp(twist.angular.y, -cap, cap);
            twist.angular.z = Mathf.Clamp(twist.angular.z, -cap, cap);
        }
    }
}
