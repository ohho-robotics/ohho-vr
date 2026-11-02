using System;
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using OmniBot.VR.Control;
using OmniBot.VR.Control.Drive;
using OmniBot.VR.Control.Session;
using OmniBot.VR.Core;
using OmniBot.VR.Core.Platform;
using OmniBot.VR.UI;

namespace OmniBot.VR.Tests
{
    /// <summary>
    /// Unit tests for teleoperation drive schemes: deadman grip enforcement
    /// (on by default), latched emergency stop, the hard 0.2 m/s turbo cap,
    /// and control-mode transitions.
    /// </summary>
    public class TeleopControlTests
    {
        /// <summary>
        /// In-memory fake transport for testing drive schemes without WebSocket networking.
        /// Nested so it does not collide with <see cref="FakeRobotLink"/> from main.
        /// </summary>
        private sealed class FakeRobotLink : IRobotLink
        {
            public bool IsConnected { get; set; } = true;
            public readonly List<(string topic, object msg)> Published = new List<(string, object)>();
            public readonly List<(string topic, string type)> Advertised = new List<(string, string)>();

            public void Advertise(string topic, string type) => Advertised.Add((topic, type));

            public void Publish<T>(string topic, T msg) => Published.Add((topic, msg));

            public void Subscribe(string topic, string type, Action<string> callback) { }

            public void Unsubscribe(string topic) { }

            public void CallService(string service, string type, Action<bool, string> onResponse) { }

            public void Clear() => Published.Clear();
        }

        private RobotProfile MakeOmniBotProfile()
        {
            return new RobotProfile
            {
                RobotId = "omnibot-pro",
                Name = "OmniBot Pro",
                Drive = DriveKind.Mecanum,
                BaseDof = 3,
                // SafetyCaps drops vy unless the profile is holonomic with a
                // lateral cap, the same fields the mecanum spec and factory set.
                // The diagonal test needs that strafe so the magnitude clamp is visible.
                Holonomic = true,
                MaxLinVel = 0.2f,
                MaxLatVel = 0.2f,
                MaxAngVel = 1.0f,
                HasArm = true,
                ArmDof = 6,
                Topics = new RobotTopics
                {
                    CmdVel = "/cmd_vel/teleop",
                    ControlMode = "/control_mode",
                    EmergencyStop = "/emergency_stop"
                }
            };
        }

        private FakeRobotLink _link;
        private MecanumDriveScheme _scheme;
        private RobotProfile _profile;

        [SetUp]
        public void SetUp()
        {
            _link = new FakeRobotLink();
            _profile = MakeOmniBotProfile();
            _scheme = new MecanumDriveScheme();
            _scheme.Configure(_profile);
        }

        [Test]
        public void EmergencyStop_BothGripsPressed_ActivatesEStopAndPublishesZeroTwist()
        {
            var input = new TeleopInput
            {
                BothGrips = true,
                BothGripsPressed = true,
                LeftGrip = true,
                RightGrip = true,
                DeltaTime = 0.05f
            };

            _scheme.Tick(ref input, _link);

            Assert.IsTrue(_scheme.IsEmergencyStopActive);

            // Verify /emergency_stop published true
            var eStopMsg = _link.Published.Find(p => p.topic == _profile.Topics.EmergencyStop);
            Assert.IsNotNull(eStopMsg.msg, "EmergencyStop topic should be published");
            Assert.IsTrue(((BoolMsg)eStopMsg.msg).data);

            // Verify zero twist published to cmd_vel
            var cmdVelMsg = _link.Published.Find(p => p.topic == _profile.Topics.CmdVel);
            Assert.IsNotNull(cmdVelMsg.msg, "CmdVel zero twist should be published on E-stop");
            var twist = (TwistMsg)cmdVelMsg.msg;
            Assert.AreEqual(0f, twist.linear.x, 0.0001f);
            Assert.AreEqual(0f, twist.linear.y, 0.0001f);
            Assert.AreEqual(0f, twist.angular.z, 0.0001f);
        }

        [Test]
        public void EmergencyStop_BothGrips_LatchesUntilExplicitReset()
        {
            // Both grips while the stick is deflected. That chord is e-stop,
            // not deadman-plus-turbo.
            var pressInput = new TeleopInput
            {
                BothGrips = true,
                BothGripsPressed = true,
                LeftGrip = true,
                RightGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref pressInput, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive);

            _link.Clear();

            // Reset is ignored while the e-stop gesture is still held.
            var resetWhileHeld = new TeleopInput
            {
                BothGrips = true,
                LeftGrip = true,
                RightGrip = true,
                ResetEmergencyStop = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref resetWhileHeld, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive, "E-stop stays latched while both grips are held");

            _link.Clear();

            // Releasing to the right grip alone would otherwise be deadman + turbo.
            var releaseToTurbo = new TeleopInput
            {
                BothGrips = false,
                BothGripsReleased = true,
                LeftGrip = false,
                RightGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref releaseToTurbo, _link);

            Assert.IsTrue(_scheme.IsEmergencyStopActive,
                "Releasing the grips must not clear e-stop; one grip is the deadman and right grip is turbo");
            foreach (var (topic, msg) in _link.Published)
            {
                if (topic == _profile.Topics.CmdVel && msg is TwistMsg twist)
                {
                    Assert.AreEqual(0f, twist.linear.x, 0.0001f);
                    Assert.AreEqual(0f, twist.linear.y, 0.0001f);
                    Assert.AreEqual(0f, twist.angular.z, 0.0001f);
                }
            }

            _link.Clear();

            // Reset while the deadman is held does not resume driving.
            var resetInput = new TeleopInput
            {
                ResetEmergencyStop = true,
                LeftGrip = true,
                RightGrip = false,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref resetInput, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive, "Reset alone does not clear e-stop");

            var releaseDeadman = new TeleopInput
            {
                LeftGrip = false,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref releaseDeadman, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive, "Releasing the deadman does not clear e-stop by itself");

            _link.Clear();
            var repress = new TeleopInput
            {
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref repress, _link);

            Assert.IsFalse(_scheme.IsEmergencyStopActive);
            var cleared = _link.Published.Find(p => p.topic == _profile.Topics.EmergencyStop);
            Assert.IsNotNull(cleared.msg, "Deadman re-press after Reset publishes /emergency_stop false");
            Assert.IsFalse(((BoolMsg)cleared.msg).data);

            var cmdVel = _link.Published.Find(p => p.topic == _profile.Topics.CmdVel);
            Assert.IsNotNull(cmdVel.msg, "Driving resumes only after Reset plus a deadman release and re-press");
            Assert.Greater(((TwistMsg)cmdVel.msg).linear.x, 0f);
            Assert.LessOrEqual(((TwistMsg)cmdVel.msg).linear.x, 0.2f);
        }

        [Test]
        public void EmergencyStop_WhileActive_BlocksAllDriveCommands()
        {
            _scheme.TriggerEmergencyStop(_link);
            _link.Clear();

            // Provide maximum thumbstick deflection while e-stop is active
            var input = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                RightStick = new Vector2(1f, 0f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref input, _link);

            // No non-zero velocity commands should be published
            foreach (var (topic, msg) in _link.Published)
            {
                if (topic == _profile.Topics.CmdVel && msg is TwistMsg twist)
                {
                    Assert.AreEqual(0f, twist.linear.x, "Linear X must be 0 while E-stop is active");
                    Assert.AreEqual(0f, twist.linear.y, "Linear Y must be 0 while E-stop is active");
                    Assert.AreEqual(0f, twist.angular.z, "Angular Z must be 0 while E-stop is active");
                }
            }
        }

        [Test]
        public void EmergencyStop_ProgrammaticTriggerAndClear()
        {
            _scheme.TriggerEmergencyStop(_link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive);

            var eStopMsg = _link.Published.Find(p => p.topic == _profile.Topics.EmergencyStop);
            Assert.IsTrue(((BoolMsg)eStopMsg.msg).data);

            _link.Clear();
            _scheme.ClearEmergencyStop(_link);
            Assert.IsFalse(_scheme.IsEmergencyStopActive);

            var clearMsg = _link.Published.Find(p => p.topic == _profile.Topics.EmergencyStop);
            Assert.IsFalse(((BoolMsg)clearMsg.msg).data);
        }

        [Test]
        public void Deadman_WhenRequiredAndGripNotHeld_BlocksMotion()
        {
            _scheme.RequireDeadman = true;

            // Deflect thumbstick forward but do not hold any grip
            var input = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                LeftGrip = false,
                RightGrip = false,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref input, _link);

            // Should NOT publish drive velocity
            var cmdVel = _link.Published.Find(p => p.topic == _profile.Topics.CmdVel);
            Assert.IsNull(cmdVel.msg, "Should not publish driving command without deadman grip");
        }

        [Test]
        public void Deadman_WhenRequiredAndGripHeld_PublishesVelocity()
        {
            _scheme.RequireDeadman = true;

            // Deflect thumbstick forward AND hold left grip (deadman)
            var input = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                LeftGrip = true,
                RightGrip = false,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref input, _link);

            var cmdVel = _link.Published.Find(p => p.topic == _profile.Topics.CmdVel);
            Assert.IsNotNull(cmdVel.msg, "Should publish driving command when deadman grip is held");
            var twist = (TwistMsg)cmdVel.msg;
            Assert.Greater(twist.linear.x, 0f, "Forward velocity should be positive");
            Assert.LessOrEqual(twist.linear.x, _profile.MaxLinVel);
        }

        [Test]
        public void Deadman_WhenReleasedWhileDriving_ImmediatelyHaltsAndPublishesZero()
        {
            _scheme.RequireDeadman = true;

            // Frame 1: driving with deadman held
            var input1 = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                LeftGrip = true,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref input1, _link);
            Assert.IsTrue(_link.Published.Exists(p => p.topic == _profile.Topics.CmdVel));

            _link.Clear();

            // Frame 2: operator releases deadman grip while still holding stick
            var input2 = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                LeftGrip = false,
                DeltaTime = 0.05f
            };
            _scheme.Tick(ref input2, _link);

            // Frame 2 must immediately publish zero twist to halt robot
            var zeroCmd = _link.Published.Find(p => p.topic == _profile.Topics.CmdVel);
            Assert.IsNotNull(zeroCmd.msg, "Zero velocity must be published immediately on deadman release");
            var twist = (TwistMsg)zeroCmd.msg;
            Assert.AreEqual(0f, twist.linear.x, 0.0001f);
            Assert.AreEqual(0f, twist.linear.y, 0.0001f);
            Assert.AreEqual(0f, twist.angular.z, 0.0001f);
        }

        [Test]
        public void Deadman_BothGripsHeld_TriggersEStopInsteadOfDrive()
        {
            _scheme.RequireDeadman = true;

            var input = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                LeftGrip = true,
                RightGrip = true,
                BothGrips = true,
                BothGripsPressed = true,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref input, _link);

            Assert.IsTrue(_scheme.IsEmergencyStopActive);
            var eStopMsg = _link.Published.Find(p => p.topic == _profile.Topics.EmergencyStop);
            Assert.IsNotNull(eStopMsg.msg);
            Assert.IsTrue(((BoolMsg)eStopMsg.msg).data);
        }

        [Test]
        public void Deadman_DefaultsToOn_OnProfileSchemeAndFactory()
        {
            var fresh = new RobotProfile();
            Assert.IsTrue(fresh.RequireDeadman, "A new profile requires the deadman");

            var unconfigured = new MecanumDriveScheme();
            Assert.IsTrue(unconfigured.RequireDeadman, "A new drive scheme requires the deadman before Configure");

            Assert.IsTrue(_profile.RequireDeadman);
            Assert.IsTrue(_scheme.RequireDeadman, "Configure copies the profile default (on)");

            var derived = RobotProfileFactory.FromGarageRobot(new GarageRobot
            {
                RobotType = new VrRobotType { Id = "mobile" },
                Category = new VrCategory { Id = "wheeled" },
                HardwareModel = new VrHardwareModel
                {
                    Id = "fast-base",
                    Name = "Fast Base",
                    Locomotion = "differential",
                    Specs = new Dictionary<string, string> { { "speed", "5 m/s" } }
                }
            });
            Assert.IsNotNull(derived);
            Assert.IsTrue(derived.RequireDeadman, "Catalog profiles keep the deadman on");
            Assert.AreEqual(SessionKind.Real, derived.Kind);
            Assert.Greater(derived.MaxLinVel, DriveSchemeBase.HardTurboLinCapMps,
                "A fast catalog speed must not be what disables the deadman");
            Assert.Greater(derived.HardMaxLinVel, 0f);
            Assert.Greater(derived.HardMaxAngVel, 0f);

            var omni = RobotProfileFactory.FromGarageRobot(new GarageRobot
            {
                RobotType = new VrRobotType { Id = "mobile" },
                Category = new VrCategory { Id = "mobile-manipulator" },
                HardwareModel = new VrHardwareModel
                {
                    Id = "omnibot-pro",
                    Name = "OmniBot Pro",
                    Locomotion = "mecanum",
                    HasArm = true,
                }
            });
            Assert.IsNotNull(omni);
            Assert.AreEqual(RobotConfig.MAX_LINEAR_VEL, omni.HardMaxLinVel, 0.0001f);
            Assert.AreEqual(RobotConfig.MAX_ANGULAR_VEL, omni.HardMaxAngVel, 0.0001f);
            Assert.IsTrue(omni.RequireDeadman);
            Assert.AreEqual(SessionKind.Real, omni.Kind);
            StringAssert.Contains("Deadman ON", _scheme.HudHint);
            StringAssert.DoesNotContain("Release to clear", OnboardingController.SafetyHint);
            StringAssert.Contains("0.2 m/s", OnboardingController.SafetyHint);
            StringAssert.Contains("latches", OnboardingController.SafetyHint);

            var blocked = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                LeftGrip = false,
                RightGrip = false,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref blocked, _link);
            var cmdVel = _link.Published.Find(p => p.topic == _profile.Topics.CmdVel);
            Assert.IsNull(cmdVel.msg, "Default deadman blocks driving when no grip is held");
        }

        [Test]
        public void Deadman_WhenExplicitlyDisabledForSimulation_AllowsDrivingWithoutGrips()
        {
            Assert.IsTrue(RobotProfile.DeadmanOptOutAllowed,
                "EditMode runs in the editor, where the simulation opt-out is allowed");
            _profile.Kind = SessionKind.Sim;
            _profile.DisableDeadmanForSimulation();
            _scheme.Configure(_profile);
            Assert.IsFalse(_profile.RequireDeadman);
            Assert.IsFalse(_scheme.RequireDeadman);

            var input = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                LeftGrip = false,
                RightGrip = false,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref input, _link);

            var cmdVel = _link.Published.Find(p => p.topic == _profile.Topics.CmdVel);
            Assert.IsNotNull(cmdVel.msg);
            Assert.Greater(((TwistMsg)cmdVel.msg).linear.x, 0f);
        }

        [Test]
        public void Deadman_RealSession_IgnoresOptOut()
        {
            Assert.AreEqual(SessionKind.Real, _profile.Kind);
            _profile.RequireDeadman = false;
            _profile.DisableDeadmanForSimulation();
            _scheme.RequireDeadman = false;
            _scheme.Configure(_profile);
            Assert.IsTrue(_profile.RequireDeadman, "A real session cannot turn the deadman off");
            Assert.IsTrue(_scheme.RequireDeadman);
            StringAssert.Contains("Deadman ON", _scheme.HudHint);
        }

        [Test]
        public void EmergencyStop_AAndB_ActivatesEStopAndPublishesZeroTwist()
        {
            var input = new TeleopInput
            {
                ButtonA = true,
                ButtonB = true,
                EmergencyStopPressed = true,
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.05f
            };
            _scheme.Tick(ref input, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive);
            var eStopMsg = _link.Published.Find(p => p.topic == _profile.Topics.EmergencyStop);
            Assert.IsTrue(((BoolMsg)eStopMsg.msg).data);
            var twist = (TwistMsg)_link.Published.Find(p => p.topic == _profile.Topics.CmdVel).msg;
            Assert.AreEqual(0f, twist.linear.x, 0.0001f);
        }

        [Test]
        public void EmergencyStop_HudButton_LatchesUntilDeadmanCycle()
        {
            var hud = new TeleopInput
            {
                HudEmergencyStop = true,
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref hud, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive);

            var release = new TeleopInput
            {
                HudEmergencyStop = false,
                LeftGrip = false,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref release, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive, "Releasing the HUD button does not clear e-stop");
        }

        [Test]
        public void TurboMode_WithRightGripAlone_AppliesMultiplierClampedToMax()
        {
            Assert.AreEqual(SessionKind.Real, _profile.Kind);
            var normalInput = new TeleopInput
            {
                LeftStick = new Vector2(0f, 0.5f),
                LeftGrip = true,
                RightGrip = false,
                RightTrigger = false,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref normalInput, _link);
            var normalTwist = (TwistMsg)_link.Published.Find(p => p.topic == _profile.Topics.CmdVel).msg;

            _link.Clear();
            var rightGripAlone = new TeleopInput
            {
                LeftStick = new Vector2(0f, 0.5f),
                LeftGrip = false,
                RightGrip = true,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref rightGripAlone, _link);
            Assert.IsFalse(_scheme.IsEmergencyStopActive, "Right grip alone is not the e-stop chord");
            var rightOnly = _link.Published.Find(p => p.topic == _profile.Topics.CmdVel);
            if (rightOnly.msg is TwistMsg rightTwist)
            {
                Assert.AreEqual(0f, rightTwist.linear.x, 0.0001f, "Right grip is not the ground deadman");
                Assert.AreEqual(0f, rightTwist.linear.y, 0.0001f);
                Assert.AreEqual(0f, rightTwist.angular.z, 0.0001f);
            }

            _link.Clear();
            var realTrigger = new TeleopInput
            {
                LeftStick = new Vector2(0f, 0.5f),
                LeftGrip = true,
                RightTrigger = true,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref realTrigger, _link);
            var realTwist = (TwistMsg)_link.Published.Find(p => p.topic == _profile.Topics.CmdVel).msg;
            Assert.AreEqual(normalTwist.linear.x, realTwist.linear.x, 0.0001f,
                "Turbo is disabled on a real robot");
            Assert.LessOrEqual(realTwist.linear.x, 0.2f);
            StringAssert.Contains("Turbo disabled", _scheme.HudHint);

            _profile.Kind = SessionKind.Sim;
            _scheme.Configure(_profile);
            _link.Clear();
            var simNormal = new TeleopInput
            {
                LeftStick = new Vector2(0f, 0.5f),
                LeftGrip = true,
                RightTrigger = false,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref simNormal, _link);
            var simBase = (TwistMsg)_link.Published.Find(p => p.topic == _profile.Topics.CmdVel).msg;
            _link.Clear();
            var simTurbo = new TeleopInput
            {
                LeftStick = new Vector2(0f, 0.5f),
                LeftGrip = true,
                RightTrigger = true,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref simTurbo, _link);
            var simTwist = (TwistMsg)_link.Published.Find(p => p.topic == _profile.Topics.CmdVel).msg;
            Assert.Greater(simTwist.linear.x, simBase.linear.x, "Sim turbo should produce higher speed");
            Assert.LessOrEqual(simTwist.linear.x, 0.2f, "Turbo must not exceed the hard 0.2 m/s cap");
            StringAssert.Contains("clamped to 0.2 m/s", _scheme.HudHint);
        }

        [Test]
        public void Turbo_ProfileMaxLinVelOfOne_NeverExceedsHardCap()
        {
            Assert.AreEqual(0.2f, DriveSchemeBase.HardTurboLinCapMps,
                "The turbo cap is a constant profiles cannot raise");
            Assert.AreEqual(RobotConfig.MAX_LINEAR_VEL, DriveSchemeBase.HardTurboLinCapMps);

            _profile.Kind = SessionKind.Sim;
            _profile.MaxLinVel = 1.0f;
            _profile.MaxAngVel = 5.0f;
            _scheme.Configure(_profile);

            var turboInput = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                RightStick = new Vector2(1f, 0f),
                LeftGrip = true,
                RightTrigger = true,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref turboInput, _link);

            var turboTwist = (TwistMsg)_link.Published.Find(p => p.topic == _profile.Topics.CmdVel).msg;
            Assert.Greater(turboTwist.linear.x, 0f, "Full stick with turbo still commands forward motion");
            Assert.AreEqual(0.2f, turboTwist.linear.x, 0.0001f,
                "Commanded speed is clamped to the OmniBot hard cap after turbo");
            Assert.LessOrEqual(turboTwist.linear.x, 0.2f);
            Assert.LessOrEqual(Mathf.Abs(turboTwist.linear.y), 0.2f);
            Assert.LessOrEqual(Mathf.Abs(turboTwist.linear.z), 0.2f);
            Assert.AreEqual(1.0f, Mathf.Abs(turboTwist.angular.z), 0.0001f,
                "Yaw is clamped to the OmniBot hard angular cap");
            Assert.LessOrEqual(Mathf.Abs(turboTwist.angular.z), RobotConfig.MAX_ANGULAR_VEL);

            // A profile slower than the constant keeps its own lower limit.
            _link.Clear();
            _profile.MaxLinVel = 0.1f;
            _scheme.Configure(_profile);
            _scheme.Tick(ref turboInput, _link);
            var slower = (TwistMsg)_link.Published.Find(p => p.topic == _profile.Topics.CmdVel).msg;
            Assert.AreEqual(0.1f, slower.linear.x, 0.0001f,
                "Turbo speed stays at the profile max when that max is below 0.2 m/s");
            Assert.LessOrEqual(slower.linear.x, 0.2f);
        }

        [Test]
        public void ControlMode_SwitchesToTeleopWhileDriving_AndNav2AfterIdleTimeout()
        {
            // Frame 1: driving
            var drivingInput = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                LeftGrip = true,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref drivingInput, _link);

            var modeMsg = _link.Published.Find(p => p.topic == _profile.Topics.ControlMode);
            Assert.IsNotNull(modeMsg.msg);
            Assert.AreEqual("teleop", ((StringMsg)modeMsg.msg).data);

            _link.Clear();

            // Advance idle time beyond 0.5s timeout (NavModeRevertDelay)
            var idleInput = new TeleopInput
            {
                LeftStick = Vector2.zero,
                DeltaTime = 0.6f
            };
            _scheme.Tick(ref idleInput, _link);

            var revertMsg = _link.Published.Find(p => p.topic == _profile.Topics.ControlMode);
            Assert.IsNotNull(revertMsg.msg);
            Assert.AreEqual("nav2", ((StringMsg)revertMsg.msg).data);
        }

        [Test]
        public void Stop_PublishesZeroTwistAndNav2Mode()
        {
            // Drive first to enter teleop mode
            var drivingInput = new TeleopInput
            {
                LeftStick = new Vector2(0f, 1f),
                LeftGrip = true,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref drivingInput, _link);
            _link.Clear();

            // Stop
            _scheme.Stop(_link);

            var cmdVel = _link.Published.Find(p => p.topic == _profile.Topics.CmdVel);
            Assert.IsNotNull(cmdVel.msg, "Stop must publish zero twist");
            var twist = (TwistMsg)cmdVel.msg;
            Assert.AreEqual(0f, twist.linear.x, 0.0001f);

            var mode = _link.Published.Find(p => p.topic == _profile.Topics.ControlMode);
            Assert.IsNotNull(mode.msg, "Stop must publish idle control mode");
            Assert.AreEqual("nav2", ((StringMsg)mode.msg).data);
        }

        [Test]
        public void ClampLinearSpeed_FullDiagonal_CapsVectorMagnitude()
        {
            var input = new TeleopInput
            {
                LeftStick = new Vector2(1f, 1f),
                LeftGrip = true,
                RightGrip = false,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref input, _link);

            var twist = (TwistMsg)_link.Published.Find(p => p.topic == _profile.Topics.CmdVel).msg;
            float mag = Mathf.Sqrt(
                twist.linear.x * twist.linear.x +
                twist.linear.y * twist.linear.y +
                twist.linear.z * twist.linear.z);
            Assert.AreEqual(_profile.MaxLinVel, mag, 0.0001f, "Diagonal stick speed is the cap, not the cap on each axis");
            Assert.Greater(twist.linear.x, 0f);
            Assert.Greater(twist.linear.y, 0f);
            Assert.Less(twist.linear.x, _profile.MaxLinVel);
        }

        [Test]
        public void StopAndConfigure_KeepLatch_AndDoNotPublishClear()
        {
            _scheme.TriggerEmergencyStop(_link);
            _link.Clear();

            _scheme.Stop(_link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive, "Stop leaves the latch set");
            Assert.IsFalse(PublishedEStop(false), "Stop must not publish emergency_stop false");

            _scheme.Configure(_profile);
            Assert.IsTrue(_scheme.IsEmergencyStopActive, "Configure must not drop the latch");
            Assert.IsFalse(PublishedEStop(false));

            var replacement = new MecanumDriveScheme();
            replacement.Configure(_profile);
            Assert.IsFalse(replacement.IsEmergencyStopActive, "A new scheme starts clear");
            replacement.RestoreEmergencyStopLatch();
            Assert.IsTrue(replacement.IsEmergencyStopActive);
            Assert.IsFalse(PublishedEStop(false), "Restoring the latch must not publish a clear");

            FinishDeadmanClear(replacement);
            Assert.IsFalse(replacement.IsEmergencyStopActive);
            Assert.AreEqual(1, CountEStop(false), "Only the deadman re-press after Reset publishes false");
        }

        [Test]
        public void TeleopRestart_OtherEndpoint_DoesNotRestoreLatch()
        {
            var profile = MakeOmniBotProfile();
            profile.HasArm = false;
            profile.ArmDof = 0;

            var host = new GameObject("teleop-endpoint-latch-test");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                typeof(TeleopController).GetField("_link", BindingFlags.Instance | BindingFlags.NonPublic)
                    .SetValue(teleop, _link);
                SetEndpoint(teleop, "10.0.0.1:9090");

                teleop.StartTeleop(profile);
                var first = DriveOf(teleop);
                var press = new TeleopInput
                {
                    LeftStick = new Vector2(0f, 1f),
                    LeftGrip = true,
                    RightGrip = true,
                    BothGrips = true,
                    BothGripsPressed = true,
                    DeltaTime = 0.1f
                };
                first.Tick(ref press, _link);
                Assert.IsTrue(first.IsEmergencyStopActive);
                _link.Clear();

                teleop.StartTeleop(profile);
                var second = DriveOf(teleop);
                Assert.AreNotSame(first, second);
                Assert.IsTrue(second.IsEmergencyStopActive, "The same endpoint keeps the latch");
                Assert.AreEqual(0, CountEStop(false), "Restart on the same endpoint must not publish false");

                SetEndpoint(teleop, "10.0.0.2:9090");
                teleop.StartTeleop(profile);
                var third = DriveOf(teleop);
                Assert.IsFalse(third.IsEmergencyStopActive,
                    "A different endpoint must not inherit the latch");
                Assert.AreEqual(0, CountEStop(false),
                    "Skipping the restore must not publish emergency_stop false");

                var drive = new TeleopInput
                {
                    LeftStick = new Vector2(0f, 1f),
                    LeftGrip = true,
                    DeltaTime = 0.1f
                };
                third.Tick(ref drive, _link);
                Assert.IsTrue(_link.Published.Exists(p =>
                    p.topic == profile.Topics.CmdVel && p.msg is TwistMsg twist && twist.linear.x > 0f),
                    "The new endpoint can drive without an explicit reset");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        [Test]
        public void EndpointChange_DropsLiveLatchWithoutPublishingClear()
        {
            var profile = MakeOmniBotProfile();
            profile.HasArm = false;
            profile.ArmDof = 0;

            var host = new GameObject("teleop-live-endpoint-test");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                typeof(TeleopController).GetField("_link", BindingFlags.Instance | BindingFlags.NonPublic)
                    .SetValue(teleop, _link);
                SetEndpoint(teleop, "10.0.0.1:9090");

                teleop.StartTeleop(profile);
                var first = DriveOf(teleop);
                var press = new TeleopInput
                {
                    BothGrips = true,
                    BothGripsPressed = true,
                    DeltaTime = 0.1f
                };
                first.Tick(ref press, _link);
                _link.Clear();

                // Stop/start records the endpoint the latch belongs to.
                teleop.StartTeleop(profile);
                var live = DriveOf(teleop);
                Assert.IsTrue(live.IsEmergencyStopActive);
                _link.Clear();

                SetEndpoint(teleop, "10.0.0.2:9090");
                typeof(TeleopController).GetMethod(
                        "ReleaseLatchIfEndpointChanged",
                        BindingFlags.Instance | BindingFlags.NonPublic)
                    .Invoke(teleop, null);

                Assert.IsFalse(live.IsEmergencyStopActive,
                    "The live scheme drops a latch that belongs to another endpoint");
                Assert.AreEqual(0, CountEStop(false),
                    "Dropping the latch must not publish emergency_stop false");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        void SetEndpoint(TeleopController teleop, string key)
        {
            typeof(TeleopController).GetField("_endpointKeyOverride", BindingFlags.Instance | BindingFlags.NonPublic)
                .SetValue(teleop, key);
        }

        [Test]
        public void TeleopRestart_KeepsLatchUntilExplicitReset()
        {
            var profile = MakeOmniBotProfile();
            profile.HasArm = false;
            profile.ArmDof = 0;

            var host = new GameObject("teleop-latch-test");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                typeof(TeleopController).GetField("_link", BindingFlags.Instance | BindingFlags.NonPublic)
                    .SetValue(teleop, _link);

                teleop.StartTeleop(profile);
                var first = DriveOf(teleop);
                var press = new TeleopInput
                {
                    LeftStick = new Vector2(0f, 1f),
                    LeftGrip = true,
                    RightGrip = true,
                    BothGrips = true,
                    BothGripsPressed = true,
                    DeltaTime = 0.1f
                };
                first.Tick(ref press, _link);
                Assert.IsTrue(first.IsEmergencyStopActive);
                _link.Clear();

                teleop.StartTeleop(profile);
                var second = DriveOf(teleop);
                Assert.AreNotSame(first, second);
                Assert.IsTrue(second.IsEmergencyStopActive, "Restart keeps the latch the robot still holds");
                Assert.IsFalse(PublishedEStop(false), "Restart must not publish emergency_stop false");

                var blocked = new TeleopInput
                {
                    LeftStick = new Vector2(0f, 1f),
                    LeftGrip = true,
                    RightGrip = false,
                    DeltaTime = 0.1f
                };
                second.Tick(ref blocked, _link);
                Assert.IsFalse(_link.Published.Exists(p =>
                    p.topic == profile.Topics.CmdVel && p.msg is TwistMsg twist && twist.linear.x > 0f));

                FinishDeadmanClear(second);
                Assert.IsFalse(second.IsEmergencyStopActive);
                Assert.AreEqual(1, CountEStop(false));
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        void FinishDeadmanClear(DriveSchemeBase scheme)
        {
            var reset = new TeleopInput { ResetEmergencyStop = true, LeftGrip = true, DeltaTime = 0.1f };
            scheme.Tick(ref reset, _link);
            Assert.IsTrue(scheme.IsEmergencyStopActive, "Reset alone does not clear e-stop");
            var release = new TeleopInput { LeftGrip = false, DeltaTime = 0.1f };
            scheme.Tick(ref release, _link);
            Assert.IsTrue(scheme.IsEmergencyStopActive, "Releasing the deadman does not clear e-stop");
            var repress = new TeleopInput
            {
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            scheme.Tick(ref repress, _link);
        }

        DriveSchemeBase DriveOf(TeleopController teleop)
        {
            var drive = typeof(TeleopController).GetField("_drive", BindingFlags.Instance | BindingFlags.NonPublic)
                .GetValue(teleop) as DriveSchemeBase;
            Assert.IsNotNull(drive);
            return drive;
        }

        bool PublishedEStop(bool value) => CountEStop(value) > 0;

        int CountEStop(bool value)
        {
            int n = 0;
            foreach (var (topic, msg) in _link.Published)
            {
                if (topic == _profile.Topics.EmergencyStop && msg is BoolMsg flag && flag.data == value)
                    n++;
            }
            return n;
        }
    }
}
