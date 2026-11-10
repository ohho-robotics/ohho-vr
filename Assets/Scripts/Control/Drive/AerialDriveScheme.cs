using UnityEngine;
using OmniBot.VR.Core;

namespace OmniBot.VR.Control.Drive
{
    /// <summary>
    /// Quadrotor / hexacopter / VTOL teleop. Mode 2: left Y climbs, left X
    /// yaws nose-right, right stick translates. A grip is the deadman. Both
    /// grips are the Ohho e-stop (land on X500, motor-cut allowed on
    /// Crazyflie). Takeoff and land are <see cref="TryTakeoff"/> /
    /// <see cref="TryLand"/>, which require the confirmation phrase. There is
    /// no stick gesture and no turbo.
    ///
    /// Commands go to ohho sim-serve as the D-05 JSON message, not a ROS twist.
    /// Catalog speed caps on the profile are left as they are; the Python
    /// SafetyGate is the cap that matters.
    /// </summary>
    public sealed class AerialDriveScheme : DriveSchemeBase
    {
        public override string HudHint =>
            "Left Y: climb · Left X: yaw nose-right · Right stick: forward/right · Either grip: deadman · No turbo · Both grips, A+B, or HUD: e-stop (latched until Reset, then release and re-press the deadman) · Takeoff/Land: buttons";

        protected override string IdleMode => "offboard";

        ISimServeSender _sender;
        AerialFlightRequest _pending;
        bool _prevDeadman;
        bool _prevEstop;
        bool _estopAnnounced;
        float _timer;

        public AerialTeleopCommand LastCommand { get; private set; }
        public string LastRefusal { get; private set; }

        public void UseSender(ISimServeSender sender)
        {
            _sender = sender;
        }

        public override void Configure(RobotProfile profile)
        {
            base.Configure(profile);
            _pending = null;
            LastCommand = null;
            LastRefusal = null;
            _prevDeadman = false;
            _prevEstop = false;
            _estopAnnounced = false;
            _timer = 0f;
        }

        /// <summary>
        /// Queue a takeoff. Returns false until the confirmation phrase matches.
        /// A stick position cannot call this.
        /// </summary>
        public bool TryTakeoff(string confirmation, double altM = AerialFlightConfirmation.DefaultTakeoffAltitudeM)
        {
            if (LastCommand != null && LastCommand.Estop)
            {
                LastRefusal = AerialFlightConfirmation.ReasonEstop;
                return false;
            }
            AerialButtonResult result = AerialFlightConfirmation.Takeoff(confirmation, altM);
            if (!result.Accepted)
            {
                LastRefusal = result.Reason;
                return false;
            }
            _pending = result.Flight;
            LastRefusal = null;
            return true;
        }

        /// <summary>Queue a land. Returns false until the confirmation phrase matches.</summary>
        public bool TryLand(string confirmation)
        {
            if (LastCommand != null && LastCommand.Estop)
            {
                LastRefusal = AerialFlightConfirmation.ReasonEstop;
                return false;
            }
            AerialButtonResult result = AerialFlightConfirmation.Land(confirmation);
            if (!result.Accepted)
            {
                LastRefusal = result.Reason;
                return false;
            }
            _pending = result.Flight;
            LastRefusal = null;
            return true;
        }

        public override void Tick(ref TeleopInput input, IRobotLink link)
        {
            if (!IsConfigured || Profile == null) return;

            // Sim-serve has its own grip deadman (either grip). The Ohho e-stop
            // latches on both grips, A+B, the HUD button, or the operate-screen
            // latch. Releasing them does not clear it. A controller latch clears
            // only after Reset, a deadman release, and a re-press. Either grip
            // is that deadman. The clear is refused while the operate latch is
            // set. There is no turbo.
            RobotClassCaps.Ensure(Profile);
            // Ground Tick flushes this before handling the latch. Aerial owns Tick,
            // so a stop taken while the robot link was down is published here once
            // the link can accept it. Sim-serve still uses its own sender.
            FlushPendingEmergencyStop(link);
            HandleEmergencyStop(ref input, link);

            bool latched = EmergencyStopActive;
            float linCap = Mathf.Max(0f, Profile.MaxLinVel);
            float angCap = Mathf.Max(0f, Profile.MaxAngVel);
            if (Profile.HardMaxLinVel > 0f) linCap = Mathf.Min(linCap, Profile.HardMaxLinVel);
            if (Profile.HardMaxAngVel > 0f) angCap = Mathf.Min(angCap, Profile.HardMaxAngVel);
            var mapped = new AerialMappingInput
            {
                RobotId = Profile.RobotId,
                RobotName = Profile.Name,
                MaxLinVel = linCap,
                MaxAngVel = angCap,
                LeftX = latched ? 0f : input.LeftStick.x,
                LeftY = latched ? 0f : input.LeftStick.y,
                RightX = latched ? 0f : input.RightStick.x,
                RightY = latched ? 0f : input.RightStick.y,
                LeftGrip = latched || input.LeftGrip,
                RightGrip = latched || input.RightGrip,
                PendingFlight = latched ? null : _pending,
            };
            AerialTeleopCommand cmd = AerialDriveMapping.Map(mapped);
            if (cmd.Estop)
                _pending = null;
            LastCommand = cmd;

            if (cmd.Estop && !_estopAnnounced)
            {
                string effect = cmd.EstopEffect == AerialEstopEffect.MotorCut ? "motor-cut allowed" : "land";
                Debug.Log("[AerialDriveScheme] E-stop (" + effect + ").");
            }
            _estopAnnounced = cmd.Estop;

            _timer += input.DeltaTime;
            bool due = _timer >= PublishInterval;
            if (due)
            {
                _timer -= PublishInterval;
                if (_timer > PublishInterval) _timer = 0f;
            }
            bool changed = cmd.Estop != _prevEstop || cmd.Deadman != _prevDeadman || cmd.Flight != null;
            if ((due || changed) && _sender != null && _sender.IsConnected)
            {
                _sender.Send(SimServeAerialJson.Serialize(cmd));
                _prevEstop = cmd.Estop;
                _prevDeadman = cmd.Deadman;
                if (cmd.Flight != null) _pending = null;
            }
        }

        public override void Stop(IRobotLink link)
        {
            _pending = null;
            // A latched e-stop must still be asserted. Sending the released
            // (deadman false, estop false) message would clear it on sim-serve.
            bool latched = EmergencyStopActive;
            var released = new AerialMappingInput
            {
                RobotId = Profile != null ? Profile.RobotId : null,
                RobotName = Profile != null ? Profile.Name : null,
                MaxLinVel = Profile != null ? Profile.MaxLinVel : 0f,
                MaxAngVel = Profile != null ? Profile.MaxAngVel : 0f,
                LeftGrip = latched,
                RightGrip = latched,
            };
            AerialTeleopCommand cmd = AerialDriveMapping.Map(released);
            LastCommand = cmd;
            _prevDeadman = false;
            _prevEstop = false;
            _estopAnnounced = false;
            _timer = 0f;
            if (_sender != null && _sender.IsConnected)
                _sender.Send(SimServeAerialJson.Serialize(cmd));
        }

        /// <summary>
        /// E-stop clear uses the same deadman as flight: either grip, never both.
        /// </summary>
        protected override bool IsResetDeadmanHeld(in TeleopInput input)
        {
            return EitherGripDeadman(input);
        }

        /// <summary>
        /// Either grip is the aerial deadman. Both grips stay the e-stop chord.
        /// </summary>
        public static bool EitherGripDeadman(in TeleopInput input)
        {
            bool both = input.BothGrips || (input.LeftGrip && input.RightGrip);
            return (input.LeftGrip || input.RightGrip) && !both;
        }

        /// <summary>
        /// Stick hold after a presence drop. A right-grip-only pilot can finish
        /// the release and re-press. Both grips do not count as that re-press.
        /// </summary>
        public static void AdvancePostLossHold(
            ref bool motionHeld,
            ref bool deadmanReleased,
            in TeleopInput input,
            bool operatorPresent,
            bool safetyLatched)
        {
            if (!motionHeld || !operatorPresent || safetyLatched) return;
            if (!EitherGripDeadman(input))
                deadmanReleased = true;
            else if (deadmanReleased)
            {
                motionHeld = false;
                deadmanReleased = false;
            }
        }

        protected override TwistMsg ComputeVelocity(
            float lx, float ly, float rx, float ry, float speedMult, out bool hasInput)
        {
            // Base.Tick dead-zones first and would apply turbo. Aerial Tick does
            // not call it. This override keeps the abstract contract on the same
            // signs, still with no turbo. The grip deadman and the latched
            // e-stop live in Tick, not in this twist helper.
            _ = speedMult;
            float linCap = Profile.MaxLinVel;
            float angCap = Profile.MaxAngVel;
            if (Profile.HardMaxLinVel > 0f) linCap = Mathf.Min(linCap, Profile.HardMaxLinVel);
            if (Profile.HardMaxAngVel > 0f) angCap = Mathf.Min(angCap, Profile.HardMaxAngVel);
            AerialAxes axes = AerialDriveMapping.ScaleDeadzoned(
                lx, ly, rx, ry, linCap, angCap);
            hasInput = axes.Vx != 0f || axes.Vy != 0f || axes.Vz != 0f || axes.YawRate != 0f;
            return new TwistMsg(
                new Vector3Msg(axes.Vx, axes.Vy, axes.Vz),
                new Vector3Msg(0f, 0f, axes.YawRate));
        }
    }
}
