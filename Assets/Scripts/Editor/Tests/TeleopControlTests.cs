using System;
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using OmniBot.VR.Control;
using OmniBot.VR.Control.Drive;
using OmniBot.VR.Control.Manip;
using OmniBot.VR.Control.Preflight;
using OmniBot.VR.Control.Session;
using OmniBot.VR.Core;
using OmniBot.VR.Core.Platform;
using OmniBot.VR.Nav;
using OmniBot.VR.Policy;
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
            OperateSafetyLatch.Clear();
            NavGoalSession.Active.ResetForTests();
            _link = new FakeRobotLink();
            _profile = MakeOmniBotProfile();
            _scheme = new MecanumDriveScheme();
            _scheme.Configure(_profile);
        }

        [TearDown]
        public void TearDown()
        {
            OperateSafetyLatch.Clear();
            NavGoalSession.Active.ResetForTests();
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
        public void EmergencyStop_AAndB_HeldAcrossLinkDrop_StaysLatched()
        {
            var drive = new TeleopInput
            {
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref drive, _link);
            Assert.IsFalse(_scheme.IsEmergencyStopActive);

            _link.IsConnected = false;
            _link.Clear();
            var edge = new TeleopInput
            {
                ButtonA = true,
                ButtonB = true,
                EmergencyStopPressed = true,
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref edge, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive, "A+B edge while the link is down still latches");
            Assert.AreEqual(0, _link.Published.Count, "A down link must not publish the stop");

            _link.IsConnected = true;
            var held = new TeleopInput
            {
                ButtonA = true,
                ButtonB = true,
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref held, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive, "A+B still held after reconnect stays latched");
            Assert.IsTrue(PublishedEStop(true), "Reconnect publishes the stop the drop could not send");
            AssertNoDriving(_link);
        }

        [Test]
        public void EmergencyStop_AAndB_LevelHoldWithoutEdge_Latches()
        {
            var held = new TeleopInput
            {
                ButtonA = true,
                ButtonB = true,
                EmergencyStopPressed = false,
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref held, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive, "A held A+B chord latches without the one-frame edge");
            AssertNoDriving(_link);
        }

        [Test]
        public void EmergencyStop_ClearWhileLinkDown_DoesNotRelease()
        {
            _scheme.TriggerEmergencyStop(_link);
            _link.Clear();
            _link.IsConnected = false;

            var reset = new TeleopInput { ResetEmergencyStop = true, LeftGrip = false, DeltaTime = 0.1f };
            _scheme.Tick(ref reset, _link);
            var repress = new TeleopInput
            {
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref repress, _link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive, "A clear is not accepted while the link is down");
            Assert.AreEqual(0, _link.Published.Count);

            _link.IsConnected = true;
            FinishDeadmanClear(_scheme);
            Assert.IsFalse(_scheme.IsEmergencyStopActive);
        }

        [Test]
        public void EmergencyStop_RightGripRepress_DoesNotClearGroundLatch()
        {
            _scheme.TriggerEmergencyStop(_link);
            _link.Clear();

            var reset = new TeleopInput
            {
                ResetEmergencyStop = true,
                RightGrip = true,
                LeftGrip = false,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref reset, _link);
            var release = new TeleopInput { LeftGrip = false, RightGrip = false, DeltaTime = 0.1f };
            _scheme.Tick(ref release, _link);
            var repress = new TeleopInput
            {
                RightGrip = true,
                LeftGrip = false,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref repress, _link);

            Assert.IsTrue(_scheme.IsEmergencyStopActive, "Right grip is not the ground deadman and cannot clear e-stop");
            Assert.IsFalse(PublishedEStop(false));
            AssertNoDriving(_link);
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
            // Simulators still revert to nav2 after idle. kind=real does not;
            // that path is ControlMode_RealRobot_IdleAndStop_DoNotPublishNav2.
            NavGoalSession.Active.ResetForTests();
            _profile.DeploymentKind = RobotDeployment.Sim;
            _scheme.Configure(_profile);

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
            // Simulators publish nav2 when teleop stops. A real robot stays out
            // of nav2 until the Nav2 button or a new goal.
            NavGoalSession.Active.ResetForTests();
            _profile.DeploymentKind = RobotDeployment.Sim;
            _scheme.Configure(_profile);

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
        public void ControlMode_RealRobot_IdleAndStop_DoNotPublishNav2()
        {
            NavGoalSession.Active.ResetForTests();
            _profile.DeploymentKind = RobotDeployment.Real;
            _scheme.Configure(_profile);

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
            var idleInput = new TeleopInput
            {
                LeftStick = Vector2.zero,
                DeltaTime = 0.6f
            };
            _scheme.Tick(ref idleInput, _link);
            Assert.IsFalse(PublishedMode("nav2"), "kind=real does not publish nav2 on idle");

            _scheme.Stop(_link);
            Assert.IsFalse(PublishedMode("nav2"), "kind=real stop does not hand control_mode to nav2");
        }

        bool PublishedMode(string mode)
        {
            return _link.Published.Exists(p =>
                p.topic == _profile.Topics.ControlMode &&
                p.msg is StringMsg text &&
                text.data == mode);
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
        public void CalibratedSpeedCap_HolonomicDiagonal_UsesActiveLinVelLimit()
        {
            _profile.MaxLinVel = 0.05f;
            _profile.MaxLatVel = 0.05f;
            _scheme.Configure(_profile);

            var input = new TeleopInput
            {
                LeftStick = new Vector2(1f, 1f),
                LeftGrip = true,
                DeltaTime = 0.1f
            };
            _scheme.Tick(ref input, _link);

            var twist = (TwistMsg)_link.Published.Find(p => p.topic == _profile.Topics.CmdVel).msg;
            float mag = LinearMagnitude(twist);
            Assert.AreEqual(0.05f, mag, 0.0001f,
                "A calibration-lowered MaxLinVel caps the diagonal, not only each axis");
            Assert.Greater(twist.linear.x, 0f);
            Assert.Greater(twist.linear.y, 0f);
            Assert.Less(mag, RobotConfig.MAX_LINEAR_VEL);
        }

        [Test]
        public void CalibratedSpeedCap_ThrusterDiagonal_UsesActiveLinVelLimit()
        {
            var profile = new RobotProfile
            {
                RobotId = "bluerov",
                Name = "BlueROV",
                Drive = DriveKind.Thruster,
                Holonomic = true,
                MaxLinVel = 0.4f,
                MaxLatVel = 0.4f,
                MaxAngVel = 1.0f,
                HardMaxLinVel = 1.5f,
                HardMaxAngVel = 1.0f,
                Topics = new RobotTopics
                {
                    CmdVel = "/cmd_vel/teleop",
                    ControlMode = "/control_mode",
                    EmergencyStop = "/emergency_stop"
                }
            };
            var scheme = new ThrusterDriveScheme();
            scheme.Configure(profile);
            var input = new TeleopInput
            {
                LeftStick = new Vector2(1f, 1f),
                RightStick = new Vector2(0f, 1f),
                LeftGrip = true,
                DeltaTime = 0.1f
            };
            scheme.Tick(ref input, _link);

            var twist = (TwistMsg)_link.Published.Find(p => p.topic == profile.Topics.CmdVel).msg;
            float mag = LinearMagnitude(twist);
            Assert.AreEqual(profile.MaxLinVel, mag, 0.0001f,
                "A thruster diagonal stays at the calibrated MaxLinVel");
            Assert.Greater(twist.linear.x, 0f);
            Assert.Greater(twist.linear.y, 0f);
            Assert.Greater(twist.linear.z, 0f);
            Assert.Less(mag, profile.HardMaxLinVel);
        }

        static float LinearMagnitude(TwistMsg twist)
        {
            return Mathf.Sqrt(
                twist.linear.x * twist.linear.x +
                twist.linear.y * twist.linear.y +
                twist.linear.z * twist.linear.z);
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

        [Test]
        public void RestoreEmergencyStopLatch_KeepsUnpublishedStop()
        {
            _link.IsConnected = false;
            _scheme.TriggerEmergencyStop(_link);
            Assert.IsTrue(_scheme.IsEmergencyStopActive);
            Assert.IsTrue(_scheme.EmergencyStopPublishPending);
            Assert.AreEqual(0, _link.Published.Count);

            var replacement = new MecanumDriveScheme();
            replacement.Configure(_profile);
            replacement.RestoreEmergencyStopLatch(publishPending: true);
            Assert.IsTrue(replacement.IsEmergencyStopActive);
            Assert.IsTrue(replacement.EmergencyStopPublishPending);
            Assert.AreEqual(0, _link.Published.Count, "Restore must not publish while the link is down");

            _link.IsConnected = true;
            var drive = new TeleopInput
            {
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                ButtonA = true,
                ButtonB = true,
                DeltaTime = 0.1f
            };
            replacement.Tick(ref drive, _link);
            Assert.IsTrue(replacement.IsEmergencyStopActive);
            Assert.IsFalse(replacement.EmergencyStopPublishPending);
            Assert.IsTrue(PublishedEStop(true), "Reconnect publishes the stop the drop could not send");
            Assert.IsFalse(PublishedEStop(false));
            AssertNoDriving(_link);
        }

        [Test]
        public void TeleopRestart_OperateOnlyStop_StaysClearableByOperateReset()
        {
            PreflightGate.Allow();
            _profile.HasArm = false;
            _profile.ArmDof = 0;

            var host = new GameObject("operate-only-restore");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                teleop.SetLink(_link);
                teleop.StartTeleop(_profile);

                OperateSafetyLatch.Latch();
                var drive = new TeleopInput
                {
                    LeftGrip = true,
                    LeftStick = new Vector2(0f, 1f),
                    DeltaTime = 0.1f
                };
                var first = DriveOf(teleop);
                first.Tick(ref drive, _link);
                Assert.IsTrue(first.IsEmergencyStopActive);
                Assert.IsFalse(first.ControllerStopLatched, "An operate-bar stop is not a controller latch");

                teleop.StartTeleop(_profile);
                var second = DriveOf(teleop);
                Assert.AreNotSame(first, second);
                Assert.IsTrue(second.IsEmergencyStopActive, "Restart keeps the operate stop");
                Assert.IsFalse(second.ControllerStopLatched,
                    "Restart must not turn an operate-only stop into a controller latch");

                _link.Clear();
                Assert.IsTrue(teleop.TryResetOperateEStop());
                Assert.IsFalse(second.IsEmergencyStopActive,
                    "Operate Reset clears a restored operate-only stop");
                Assert.IsTrue(PublishedEStop(false));
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        [Test]
        public void TeleopRestart_ControllerStop_StaysControllerLatch()
        {
            PreflightGate.Allow();
            _profile.HasArm = false;
            _profile.ArmDof = 0;

            var host = new GameObject("controller-latch-restore");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                teleop.SetLink(_link);
                teleop.StartTeleop(_profile);

                var press = new TeleopInput
                {
                    BothGrips = true,
                    BothGripsPressed = true,
                    LeftGrip = true,
                    RightGrip = true,
                    DeltaTime = 0.1f
                };
                var first = DriveOf(teleop);
                first.Tick(ref press, _link);
                Assert.IsTrue(first.ControllerStopLatched);

                teleop.StartTeleop(_profile);
                var second = DriveOf(teleop);
                Assert.IsTrue(second.IsEmergencyStopActive);
                Assert.IsTrue(second.ControllerStopLatched,
                    "Restart keeps a both-grips stop as a controller latch");

                _link.Clear();
                Assert.IsTrue(teleop.TryResetOperateEStop());
                Assert.IsTrue(second.IsEmergencyStopActive,
                    "Operate Reset does not clear a restored controller stop");
                Assert.IsFalse(PublishedEStop(false),
                    "Operate Reset must not publish emergency_stop false over a restored controller latch");

                FinishDeadmanClear(second);
                Assert.IsFalse(second.IsEmergencyStopActive);
                Assert.AreEqual(1, CountEStop(false));
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        [Test]
        public void TeleopRestart_FixedBaseOperateStop_StaysClearableByOperateReset()
        {
            PreflightGate.Allow();
            var profile = MakeOmniBotProfile();
            profile.RobotId = "so101-arm";
            profile.Name = "SO-101";
            profile.Drive = DriveKind.FixedBase;
            profile.BaseDof = 0;
            profile.Holonomic = false;
            profile.HasArm = true;
            profile.ArmDof = 6;
            profile.Topics.CmdVel = "";
            profile.Topics.EmergencyStop = "/emergency_stop";

            var host = new GameObject("fixed-base-operate-restore");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                teleop.SetLink(_link);
                teleop.StartTeleop(profile);

                OperateSafetyLatch.Latch();
                Pump(teleop, new TeleopInput { LeftGrip = true, DeltaTime = 0.1f }, true, false);
                Assert.IsTrue(teleop.IsGripEStopActive, "The operate bar latches a fixed-base arm");

                teleop.StartTeleop(profile);
                Assert.IsTrue(teleop.IsGripEStopActive, "Restart keeps the operate stop");

                _link.Clear();
                Assert.IsTrue(teleop.TryResetOperateEStop());
                Assert.IsFalse(teleop.IsGripEStopActive,
                    "Operate Reset clears a restored operate-only arm stop");
                Assert.IsTrue(_link.Published.Exists(p =>
                    p.topic == profile.Topics.EmergencyStop && p.msg is BoolMsg flag && !flag.data));
            }
            finally
            {
                if (host != null)
                {
                    var teleop = host.GetComponent<TeleopController>();
                    if (teleop != null) teleop.StopTeleop();
                }
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        [Test]
        public void TeleopRestart_LinkDownEStop_PublishesOnReconnect_ForEverySource()
        {
            AssertPendingSurvivesRestart(new TeleopInput
            {
                ButtonA = true,
                ButtonB = true,
                EmergencyStopPressed = true,
                DeltaTime = 0.1f
            }, "A+B");
            AssertPendingSurvivesRestart(new TeleopInput
            {
                HudEmergencyStop = true,
                DeltaTime = 0.1f
            }, "HUD");
            AssertPendingSurvivesRestart(new TeleopInput
            {
                LeftGrip = true,
                RightGrip = true,
                BothGrips = true,
                BothGripsPressed = true,
                DeltaTime = 0.1f
            }, "both grips");
        }

        [Test]
        public void TeleopRestart_AfterDeliveredEStop_DoesNotRepublish()
        {
            _profile.HasArm = false;
            _profile.ArmDof = 0;
            var host = new GameObject("delivered-estop-restart");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                typeof(TeleopController).GetField("_link", BindingFlags.Instance | BindingFlags.NonPublic)
                    .SetValue(teleop, _link);
                SetEndpoint(teleop, "10.0.0.9:9090");
                PreflightGate.Allow();

                teleop.StartTeleop(_profile);
                var first = DriveOf(teleop);
                var stop = new TeleopInput
                {
                    ButtonA = true,
                    ButtonB = true,
                    EmergencyStopPressed = true,
                    DeltaTime = 0.1f
                };
                first.Tick(ref stop, _link);
                Assert.IsTrue(first.IsEmergencyStopActive);
                Assert.IsFalse(first.EmergencyStopPublishPending, "A delivered stop is not waiting to publish");
                _link.Clear();

                teleop.StartTeleop(_profile);
                var second = DriveOf(teleop);
                Assert.IsTrue(second.IsEmergencyStopActive);
                Assert.IsFalse(second.EmergencyStopPublishPending);

                var drive = new TeleopInput
                {
                    LeftGrip = true,
                    LeftStick = new Vector2(0f, 1f),
                    DeltaTime = 0.1f
                };
                second.Tick(ref drive, _link);
                Assert.IsFalse(PublishedEStop(true), "Restart does not send emergency_stop true again");
                Assert.IsFalse(PublishedEStop(false));
                AssertNoDriving(_link);
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        void AssertPendingSurvivesRestart(TeleopInput stop, string source)
        {
            _link.Clear();
            _link.IsConnected = true;
            OperateSafetyLatch.Clear();
            _profile.HasArm = false;
            _profile.ArmDof = 0;

            var host = new GameObject("pending-estop-" + source);
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                typeof(TeleopController).GetField("_link", BindingFlags.Instance | BindingFlags.NonPublic)
                    .SetValue(teleop, _link);
                SetEndpoint(teleop, "10.0.0.9:9090");
                PreflightGate.Allow();

                teleop.StartTeleop(_profile);
                var first = DriveOf(teleop);

                _link.IsConnected = false;
                _link.Clear();
                first.Tick(ref stop, _link);
                Assert.IsTrue(first.IsEmergencyStopActive, source);
                Assert.IsTrue(first.EmergencyStopPublishPending, source);
                Assert.AreEqual(0, CountEStop(true), source);

                // Chord released. The rebuild has to keep the unpublished stop.
                teleop.StartTeleop(_profile);
                var second = DriveOf(teleop);
                Assert.AreNotSame(first, second, source);
                Assert.IsTrue(second.IsEmergencyStopActive, source);
                Assert.IsTrue(second.EmergencyStopPublishPending, source + " rebuild keeps the unpublished stop");
                Assert.AreEqual(0, CountEStop(true), source);

                _link.IsConnected = true;
                var drive = new TeleopInput
                {
                    LeftGrip = true,
                    LeftStick = new Vector2(0f, 1f),
                    DeltaTime = 0.1f
                };
                second.Tick(ref drive, _link);
                Assert.IsTrue(second.IsEmergencyStopActive, source);
                Assert.IsFalse(second.EmergencyStopPublishPending, source);
                Assert.IsTrue(PublishedEStop(true), source + " reconnect publishes the stop");
                Assert.IsFalse(PublishedEStop(false), source);
                AssertNoDriving(_link);
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

        [Test]
        public void AAndB_Hud_AndDriveLatch_DisarmTheArm()
        {
            PreflightGate.Allow();
            var profile = MakeOmniBotProfile();
            profile.Deployment = ArmDeployment.Sim;
            profile.Topics.ArmEnable = "/arm/enable";
            profile.Topics.ArmCommands = "/arm/joint_commands/teleop";

            var origin = new GameObject("arm-estop-origin");
            var host = new GameObject("arm-estop-host");
            try
            {
                var arm = new HandIK6DofScheme();
                arm.Configure(profile, origin.transform);
                var teleop = host.AddComponent<TeleopController>();
                var flags = BindingFlags.Instance | BindingFlags.NonPublic;
                typeof(TeleopController).GetField("_link", flags).SetValue(teleop, _link);
                typeof(TeleopController).GetField("_manip", flags).SetValue(teleop, arm);
                typeof(TeleopController).GetField("_drive", flags).SetValue(teleop, _scheme);

                arm.SetArmEnabled(true, _link);
                Assert.IsTrue(arm.ArmEnabled);
                SetLastInput(teleop, new TeleopInput
                {
                    ButtonA = true,
                    ButtonB = true,
                    EmergencyStopPressed = true,
                    DeltaTime = 0.1f
                });
                TickManip(teleop);
                Assert.IsFalse(arm.ArmEnabled, "A+B disarms the arm");

                ReleaseArm(teleop, arm);
                SetLastInput(teleop, new TeleopInput { HudEmergencyStop = true, DeltaTime = 0.1f });
                TickManip(teleop);
                Assert.IsFalse(arm.ArmEnabled, "HUD e-stop disarms the arm");

                ReleaseArm(teleop, arm);
                _scheme.TriggerEmergencyStop(_link);
                SetLastInput(teleop, new TeleopInput { LeftGrip = true, RightTrigger = true, DeltaTime = 0.1f });
                TickManip(teleop);
                Assert.IsFalse(arm.ArmEnabled, "A latched drive stop disarms the arm after the button is released");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(host);
                UnityEngine.Object.DestroyImmediate(origin);
            }
        }

        [Test]
        public void HudEmergencyStop_NotifiesRunningPolicy()
        {
            var profile = MakeOmniBotProfile();
            profile.PolicyModelsFallback = "act,smolvla";
            profile.Topics.ArmCmdMode = "/arm/cmd_mode";
            profile.Topics.ArmCommands = "/arm/joint_commands/teleop";
            profile.Topics.PolicyEnable = "/policy/enable";
            profile.Topics.PolicyLoad = "/policy/load";

            var policy = new PolicyController();
            policy.Bind(profile, _link, PolicySessionKind.Sim);
            Assert.IsTrue(policy.Select("act"));
            Assert.IsTrue(policy.Load().Accepted, policy.LastRefusal);
            policy.Confirm();
            Assert.IsTrue(policy.Start().Accepted, policy.LastRefusal);
            Assert.AreEqual(PolicyRunState.Running, policy.RunState);

            var host = new GameObject("hud-policy-estop");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                teleop.EmergencyStopChanged += policy.NotifyEmergencyStop;
                teleop.RequestHudEmergencyStop();

                Assert.AreEqual(PolicyRunState.Stopped, policy.RunState);
                Assert.AreEqual("estop", policy.LastStopReason);
                Assert.IsTrue(policy.EmergencyStopActive);
                Assert.IsTrue(_link.Published.Exists(p =>
                    p.topic == profile.Topics.ControlMode && p.msg is StringMsg mode && mode.data == "teleop"));

                policy.Confirm();
                Assert.AreEqual(PolicyRefuse.Estop, policy.Start().Reason);

                bool yielded = false;
                teleop.EmergencyStopChanged += active => yielded = active;
                var notify = typeof(TeleopController).GetMethod(
                    "NotifyEmergencyStopEdges",
                    BindingFlags.Instance | BindingFlags.NonPublic);
                notify.Invoke(teleop, new object[] { new TeleopInput { HudEmergencyStop = true } });
                Assert.IsTrue(yielded, "HUD e-stop on the control frame yields the policy");

                yielded = false;
                notify.Invoke(teleop, new object[] { new TeleopInput { EmergencyStopPressed = true } });
                Assert.IsTrue(yielded, "A+B on the control frame yields the policy");

                yielded = false;
                notify.Invoke(teleop, new object[] { new TeleopInput { BothGripsPressed = true } });
                Assert.IsTrue(yielded, "Both grips on the control frame yield the policy");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        void SetLastInput(TeleopController teleop, TeleopInput input)
        {
            typeof(TeleopController).GetField("_lastInput", BindingFlags.Instance | BindingFlags.NonPublic)
                .SetValue(teleop, input);
        }

        void TickManip(TeleopController teleop)
        {
            typeof(TeleopController).GetMethod(
                    "TickManipulation",
                    BindingFlags.Instance | BindingFlags.NonPublic)
                .Invoke(teleop, null);
        }

        void ReleaseArm(TeleopController teleop, HandIK6DofScheme arm)
        {
            SetLastInput(teleop, new TeleopInput { DeltaTime = 0.1f });
            TickManip(teleop);
            arm.SetArmEnabled(true, _link);
            Assert.IsTrue(arm.ArmEnabled);
        }

        static void AssertNoDriving(FakeRobotLink link)
        {
            foreach (var (topic, msg) in link.Published)
            {
                if (msg is TwistMsg twist && twist.linear != null)
                {
                    Assert.AreEqual(0f, twist.linear.x, 0.0001f, topic);
                    Assert.AreEqual(0f, twist.linear.y, 0.0001f, topic);
                    Assert.AreEqual(0f, twist.linear.z, 0.0001f, topic);
                }
            }
        }

        [Test]
        public void OperateReset_LeavesControllerEStop_UntilDeadmanSequence_AndPolicyFollowsThatLatch()
        {
            PreflightGate.Allow();
            _profile.HasArm = false;
            _profile.ArmDof = 0;
            var policyProfile = MakeOmniBotProfile();
            policyProfile.Topics.ArmCmdMode = "/arm/cmd_mode";
            policyProfile.Topics.ArmCommands = "/arm/joint_commands/teleop";
            policyProfile.Topics.PolicyEnable = "/policy/enable";
            policyProfile.Topics.PolicyLoad = "/policy/load";
            policyProfile.PolicyModelsFallback = "act,smolvla";

            var policy = new PolicyController();
            policy.Bind(policyProfile, _link, PolicySessionKind.Sim);
            Assert.IsTrue(policy.Select("act"));
            Assert.IsTrue(policy.Load().Accepted, policy.LastRefusal);
            policy.Confirm();
            Assert.IsTrue(policy.Start().Accepted, policy.LastRefusal);

            var host = new GameObject("operate-reset-estop");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                teleop.SetLink(_link);
                teleop.EmergencyStopChanged += policy.NotifyEmergencyStop;
                teleop.StartTeleop(_profile);
                var drive = DriveOf(teleop);

                Pump(teleop, new TeleopInput
                {
                    BothGrips = true,
                    BothGripsPressed = true,
                    LeftGrip = true,
                    RightGrip = true,
                    DeltaTime = 0.1f,
                }, linkUp: true, aerialUp: false);
                Assert.IsTrue(drive.IsEmergencyStopActive);
                Assert.AreEqual(PolicyRunState.Stopped, policy.RunState);
                Assert.IsTrue(policy.EmergencyStopActive);

                _link.Clear();
                Pump(teleop, new TeleopInput
                {
                    BothGripsReleased = true,
                    LeftGrip = true,
                    DeltaTime = 0.1f,
                }, linkUp: true, aerialUp: false);
                Assert.IsTrue(drive.IsEmergencyStopActive, "Releasing a grip leaves the latch set");
                Assert.IsTrue(policy.EmergencyStopActive,
                    "A grip release must not clear the policy latch while the drive is still stopped");
                Assert.IsTrue(teleop.TryResetOperateEStop());
                Assert.IsTrue(drive.IsEmergencyStopActive,
                    "Operate Reset does not clear a both-grips stop, including while the deadman is held");
                Assert.IsFalse(PublishedEStop(false));
                Assert.IsTrue(policy.EmergencyStopActive);

                int vlaBefore = CountMode("vla");
                int policyModeBefore = CountMode("policy");
                policy.Confirm();
                Assert.AreEqual(PolicyRefuse.Estop, policy.Start().Reason);
                Assert.AreEqual(vlaBefore, CountMode("vla"), "A refused start must not publish control_mode=vla");
                Assert.AreEqual(policyModeBefore, CountMode("policy"),
                    "A refused start must not publish ArmCmdMode=policy");

                Pump(teleop, new TeleopInput { ResetEmergencyStop = true, LeftGrip = true, DeltaTime = 0.1f }, true, false);
                Pump(teleop, new TeleopInput { LeftGrip = false, DeltaTime = 0.1f }, true, false);
                Assert.IsTrue(policy.EmergencyStopActive, "The policy latch stays until the deadman is pressed again");
                Pump(teleop, new TeleopInput
                {
                    LeftGrip = true,
                    LeftStick = new Vector2(0f, 1f),
                    DeltaTime = 0.1f,
                }, true, false);
                Assert.IsFalse(drive.IsEmergencyStopActive);
                Assert.IsFalse(policy.EmergencyStopActive,
                    "The policy latch clears when the drive latch clears");
                policy.Confirm();
                Assert.IsTrue(policy.Start().Accepted, policy.LastRefusal);
                Assert.Greater(CountMode("vla"), vlaBefore);
                Assert.Greater(CountMode("policy"), policyModeBefore);
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        [Test]
        public void AerialAAndB_LatchesWhileBothLinksAreDown()
        {
            PreflightGate.Allow();
            var profile = MakeOmniBotProfile();
            profile.RobotId = "px4-x500";
            profile.Name = "px4-x500";
            profile.Drive = DriveKind.Quadrotor;
            profile.HasArm = false;
            profile.ArmDof = 0;
            profile.Holonomic = true;

            var host = new GameObject("aerial-drop-estop");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                _link.IsConnected = false;
                teleop.SetLink(_link);
                teleop.StartTeleop(profile);
                var aerial = DriveOf(teleop) as AerialDriveScheme;
                Assert.IsNotNull(aerial);

                Pump(teleop, new TeleopInput
                {
                    ButtonA = true,
                    ButtonB = true,
                    EmergencyStopPressed = true,
                    DeltaTime = 0.1f,
                }, linkUp: false, aerialUp: false);
                Assert.IsTrue(aerial.IsEmergencyStopActive, "A+B while both links are down still latches");
                Assert.IsTrue(aerial.EmergencyStopPublishPending);
                Assert.AreEqual(0, _link.Published.Count);

                Pump(teleop, new TeleopInput { DeltaTime = 0.1f }, linkUp: false, aerialUp: false);
                Assert.IsTrue(aerial.IsEmergencyStopActive, "Releasing A+B during the drop keeps the latch");

                Pump(teleop, new TeleopInput { ResetEmergencyStop = true, LeftGrip = false, DeltaTime = 0.1f }, false, false);
                Pump(teleop, new TeleopInput { LeftGrip = true, DeltaTime = 0.1f }, false, false);
                Assert.IsTrue(aerial.IsEmergencyStopActive, "The clear sequence does not run while both links are down");
                Assert.AreEqual(0, _link.Published.Count);

                _link.IsConnected = true;
                _link.Clear();
                Pump(teleop, new TeleopInput { DeltaTime = 0.1f }, linkUp: true, aerialUp: false);
                Assert.IsTrue(aerial.IsEmergencyStopActive);
                Assert.IsFalse(aerial.EmergencyStopPublishPending);
                Assert.IsTrue(PublishedEStop(true), "The link publishes the stop that was latched during the drop");
                Assert.IsFalse(PublishedEStop(false));
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        [Test]
        public void FixedBaseHudAndAAndB_PublishAndLatchUntilDeadmanSequence()
        {
            PreflightGate.Allow();
            var profile = MakeOmniBotProfile();
            profile.RobotId = "so101-arm";
            profile.Name = "SO-101";
            profile.Drive = DriveKind.FixedBase;
            profile.BaseDof = 0;
            profile.Holonomic = false;
            profile.HasArm = true;
            profile.ArmDof = 6;
            profile.Deployment = ArmDeployment.Sim;
            profile.MaxLinVel = 0f;
            profile.MaxAngVel = 0f;
            profile.Topics.CmdVel = "";
            profile.Topics.EmergencyStop = "/emergency_stop";
            profile.Topics.ArmEnable = "/arm/enable";
            profile.Topics.ArmCommands = "/arm/joint_commands";
            profile.Topics.ArmCmdMode = "/arm/cmd_mode";
            profile.Topics.ControlMode = "/control_mode";
            profile.Topics.PolicyEnable = "/policy/enable";
            profile.Topics.PolicyLoad = "/policy/load";
            profile.PolicyModelsFallback = "act,smolvla";

            var policy = new PolicyController();
            policy.Bind(profile, _link, PolicySessionKind.Sim);
            Assert.IsTrue(policy.Select("act"));
            Assert.IsTrue(policy.Load().Accepted, policy.LastRefusal);
            policy.Confirm();
            Assert.IsTrue(policy.Start().Accepted, policy.LastRefusal);

            var host = new GameObject("fixed-base-estop");
            try
            {
                var teleop = host.AddComponent<TeleopController>();
                teleop.SetLink(_link);
                teleop.EmergencyStopChanged += policy.NotifyEmergencyStop;
                teleop.StartTeleop(profile);
                Assert.IsNull(typeof(TeleopController).GetField("_drive", BindingFlags.Instance | BindingFlags.NonPublic)
                    .GetValue(teleop));
                var arm = typeof(TeleopController).GetField("_manip", BindingFlags.Instance | BindingFlags.NonPublic)
                    .GetValue(teleop) as HandIK6DofScheme;
                Assert.IsNotNull(arm);
                arm.SetArmEnabled(true, _link);
                Assert.IsTrue(arm.ArmEnabled);

                _link.Clear();
                teleop.RequestHudEmergencyStop();
                Pump(teleop, new TeleopInput { DeltaTime = 0.1f }, linkUp: true, aerialUp: false);
                Assert.IsTrue(teleop.IsGripEStopActive, "HUD e-stop latches a fixed-base arm");
                Assert.IsTrue(PublishedEStop(true), "HUD e-stop publishes emergency_stop");
                Assert.IsFalse(arm.ArmEnabled, "HUD e-stop disarms the arm");
                Assert.AreEqual(PolicyRunState.Stopped, policy.RunState);

                Pump(teleop, new TeleopInput { BothGripsReleased = true, DeltaTime = 0.1f }, true, false);
                Assert.IsTrue(teleop.IsGripEStopActive, "The latch remains after the HUD press is consumed");
                Assert.IsTrue(policy.EmergencyStopActive);
                Assert.IsTrue(teleop.TryResetOperateEStop());
                Assert.IsTrue(teleop.IsGripEStopActive, "Operate Reset does not clear the HUD latch");
                Assert.IsFalse(PublishedEStop(false));

                policy.Confirm();
                Assert.AreEqual(PolicyRefuse.Estop, policy.Start().Reason);

                Pump(teleop, new TeleopInput { ResetEmergencyStop = true, LeftGrip = true, DeltaTime = 0.1f }, true, false);
                Pump(teleop, new TeleopInput { LeftGrip = false, DeltaTime = 0.1f }, true, false);
                Pump(teleop, new TeleopInput { LeftGrip = true, DeltaTime = 0.1f }, true, false);
                Assert.IsFalse(teleop.IsGripEStopActive);
                Assert.IsTrue(PublishedEStop(false));
                Assert.IsFalse(policy.EmergencyStopActive, "The policy latch clears with the arm latch");
                arm.SetArmEnabled(true, _link);
                Pump(teleop, new TeleopInput { LeftGrip = true, DeltaTime = 0.1f }, true, false);
                Assert.IsTrue(arm.ArmEnabled, "The arm can be enabled again after the deadman sequence");

                _link.IsConnected = false;
                _link.Clear();
                Pump(teleop, new TeleopInput
                {
                    ButtonA = true,
                    ButtonB = true,
                    EmergencyStopPressed = true,
                    DeltaTime = 0.1f,
                }, linkUp: false, aerialUp: false);
                Assert.IsTrue(teleop.IsGripEStopActive, "A+B while the link is down latches a fixed-base arm");
                Assert.AreEqual(0, CountEStop(true));
                Pump(teleop, new TeleopInput { DeltaTime = 0.1f }, false, false);
                Assert.IsTrue(teleop.IsGripEStopActive, "Releasing A+B during the drop keeps the latch");
                Pump(teleop, new TeleopInput { ResetEmergencyStop = true, DeltaTime = 0.1f }, false, false);
                Pump(teleop, new TeleopInput { LeftGrip = true, DeltaTime = 0.1f }, false, false);
                Assert.IsTrue(teleop.IsGripEStopActive, "The clear sequence waits until the link is up");

                _link.IsConnected = true;
                Pump(teleop, new TeleopInput { DeltaTime = 0.1f }, true, false);
                Assert.IsTrue(PublishedEStop(true), "Reconnect publishes the A+B stop");
                Assert.IsTrue(policy.EmergencyStopActive);
            }
            finally
            {
                if (host != null)
                {
                    var teleop = host.GetComponent<TeleopController>();
                    if (teleop != null) teleop.StopTeleop();
                }
                UnityEngine.Object.DestroyImmediate(host);
            }
        }

        static void Pump(TeleopController teleop, TeleopInput input, bool linkUp, bool aerialUp)
        {
            typeof(TeleopController).GetMethod("PumpFrame", BindingFlags.Instance | BindingFlags.NonPublic)
                .Invoke(teleop, new object[] { input, linkUp, aerialUp });
        }

        int CountMode(string mode)
        {
            int n = 0;
            foreach (var (topic, msg) in _link.Published)
            {
                if (msg is StringMsg text && text.data == mode)
                    n++;
            }
            return n;
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
