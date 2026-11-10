using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using OmniBot.VR.Control;
using OmniBot.VR.Control.Drive;
using OmniBot.VR.Control.Preflight;
using OmniBot.VR.Control.Safety;
using OmniBot.VR.Control.Session;
using OmniBot.VR.Core;
using OmniBot.VR.Core.Platform;
using OmniBot.VR.Nav;

namespace OmniBot.VR.Tests
{
    /// <summary>
    /// Nav2 headset goals: real robots do not resume nav2 on idle, a stick
    /// during nav2 cancels the goal, and waypoint / plan / map payloads match
    /// the rosbridge contract.
    /// </summary>
    public class NavGoalTests
    {
        private NavGoalSession _session;
        private FakeTransport _transport;
        private FakeLink _link;

        [SetUp]
        public void SetUp()
        {
            OperateSafetyLatch.Clear();
            _session = NavGoalSession.Active;
            _session.ResetForTests();
            _transport = new FakeTransport();
            _link = new FakeLink();
            _session.Transport = _transport;
        }

        [TearDown]
        public void TearDown()
        {
            OperateSafetyLatch.Clear();
            _session.ResetForTests();
        }

        // ── Idle policy ──────────────────────────────────────────────────────

        [Test]
        public void ResolveIdle_Real_SuppressesNav2()
        {
            Assert.IsNull(ControlModeMachine.ResolveIdle("nav2", "real"));
            Assert.IsNull(ControlModeMachine.ResolveIdle("nav2", "REAL"));
            Assert.IsNull(ControlModeMachine.ResolveIdle("nav2", null));
        }

        [Test]
        public void ResolveIdle_Sim_KeepsNav2()
        {
            Assert.AreEqual("nav2", ControlModeMachine.ResolveIdle("nav2", "sim"));
            Assert.AreEqual("nav2", ControlModeMachine.ResolveIdle("nav2", "gazebo"));
        }

        [Test]
        public void ResolveIdle_Real_KeepsOffboard()
        {
            Assert.AreEqual("offboard", ControlModeMachine.ResolveIdle("offboard", "real"));
        }

        [Test]
        public void ResolveIdle_Empty_IsNull()
        {
            Assert.IsNull(ControlModeMachine.ResolveIdle("", "sim"));
        }

        [Test]
        public void Deployment_ExplicitKindBeatsSimulatedStatus()
        {
            Assert.AreEqual(RobotDeployment.Real, RobotDeployment.FromGarage("simulated", "real"));
        }

        [Test]
        public void Deployment_SimulatedStatus_IsSim()
        {
            Assert.AreEqual(RobotDeployment.Sim, RobotDeployment.FromGarage("simulated", null));
            Assert.AreEqual(RobotDeployment.Sim, RobotDeployment.FromGarage("active", "gazebo"));
        }

        [Test]
        public void Deployment_Missing_IsReal()
        {
            Assert.AreEqual(RobotDeployment.Real, RobotDeployment.FromGarage("active", null));
            Assert.AreEqual(RobotDeployment.Real, RobotDeployment.FromGarage(null, "hardware"));
        }

        [Test]
        public void Machine_RealIdle_DoesNotPublishNav2()
        {
            var mode = new ControlModeMachine();
            mode.Reset("nav2", RobotDeployment.Real);
            mode.Tick(true, 0.1f);
            var idle = mode.Tick(false, 0.6f);
            Assert.IsFalse(mode.InTeleop);
            Assert.IsNull(idle.PublishMode);
            Assert.IsTrue(idle.IdleHeldTeleop);
            Assert.IsFalse(idle.CancelNavGoal);
        }

        [Test]
        public void Machine_SimIdle_PublishesNav2()
        {
            var mode = new ControlModeMachine();
            mode.Reset("nav2", RobotDeployment.Sim);
            mode.Tick(true, 0.1f);
            var early = mode.Tick(false, 0.49f);
            Assert.IsNull(early.PublishMode);
            var idle = mode.Tick(false, 0.02f);
            Assert.AreEqual("nav2", idle.PublishMode);
        }

        [Test]
        public void Machine_RealStop_DoesNotReturnNav2()
        {
            var mode = new ControlModeMachine();
            mode.Reset("nav2", RobotDeployment.Real);
            mode.Tick(true, 0.1f);
            Assert.IsNull(mode.Stop());
        }

        [Test]
        public void Machine_SimStop_ReturnsNav2()
        {
            var mode = new ControlModeMachine();
            mode.Reset("nav2", RobotDeployment.Sim);
            mode.Tick(true, 0.1f);
            Assert.AreEqual("nav2", mode.Stop());
        }

        [Test]
        public void DriveStop_Real_ZerosTwistWithoutNav2()
        {
            // Deadman defaults on. The grip is what lets the stick enter teleop;
            // a real stop from teleop still must not hand control_mode to nav2.
            var scheme = ControlSchemeFactory.CreateDriveScheme(DriveProfile(RobotDeployment.Real));
            var input = DrivingStick();
            scheme.Tick(ref input, _link);
            _link.Published.Clear();
            scheme.Stop(_link);
            Assert.IsTrue(_link.Published.Exists(p => p.topic == "/cmd_vel/teleop"));
            Assert.IsFalse(_link.Published.Exists(p => p.topic == "/control_mode" && p.json.Contains("nav2")));
        }

        [Test]
        public void DriveStop_Sim_PublishesNav2()
        {
            // Same driving precondition as the real case. Stop always publishes
            // a zero twist, then nav2 once the simulator was actually in teleop.
            var scheme = ControlSchemeFactory.CreateDriveScheme(DriveProfile(RobotDeployment.Sim));
            var input = DrivingStick();
            scheme.Tick(ref input, _link);
            _link.Published.Clear();
            scheme.Stop(_link);
            Assert.IsTrue(_link.Published.Exists(p => p.topic == "/cmd_vel/teleop"));
            Assert.IsTrue(_link.Published.Exists(p => p.topic == "/control_mode" && p.json.Contains("\"nav2\"")));
        }

        [Test]
        public void Machine_StickDuringNav2_CancelsOnce()
        {
            var mode = new ControlModeMachine();
            mode.Reset("nav2", RobotDeployment.Real);
            mode.Observe(true);
            var first = mode.Tick(true, 0.02f);
            Assert.IsTrue(first.CancelNavGoal);
            Assert.AreEqual("teleop", first.PublishMode);
            var second = mode.Tick(true, 0.02f);
            Assert.IsFalse(second.CancelNavGoal);
            Assert.IsNull(second.PublishMode);
        }

        [Test]
        public void Machine_StickDuringTeleop_DoesNotCancel()
        {
            var mode = new ControlModeMachine();
            mode.Reset("nav2", RobotDeployment.Real);
            mode.Observe(false);
            mode.Tick(true, 0.02f);
            var again = mode.Tick(true, 0.02f);
            Assert.IsFalse(again.CancelNavGoal);
            Assert.IsNull(again.PublishMode);
        }

        // ── Handover on the link ─────────────────────────────────────────────

        [Test]
        public void Handover_RealIdle_DoesNotPublishNav2()
        {
            var mode = new ControlModeMachine();
            mode.Reset("nav2", RobotDeployment.Real);
            NavStickHandover.Apply(mode, _session, _link, "/control_mode", true, 0.02f);
            _link.Published.Clear();
            NavStickHandover.Apply(mode, _session, _link, "/control_mode", false, 0.6f);
            Assert.IsFalse(_link.Published.Exists(p => p.topic == "/control_mode" && p.json.Contains("nav2")));
        }

        [Test]
        public void Handover_SimIdleThenStick_CancelsAndReturnsTeleop()
        {
            var mode = new ControlModeMachine();
            mode.Reset("nav2", RobotDeployment.Sim);
            NavStickHandover.Apply(mode, _session, _link, "/control_mode", true, 0.02f);
            _link.Published.Clear();
            NavStickHandover.Apply(mode, _session, _link, "/control_mode", false, 0.6f);
            Assert.IsTrue(_link.Published.Exists(p => p.topic == "/control_mode" && p.json.Contains("nav2")));
            Assert.IsTrue(_session.ModeIsNav2);

            _link.Published.Clear();
            NavStickHandover.Apply(mode, _session, _link, "/control_mode", true, 0.02f);
            Assert.IsTrue(_link.Published.Exists(p => p.topic == "/mission/cancel"));
            Assert.IsTrue(_link.Published.Exists(p => p.topic == "/control_mode" && p.json.Contains("teleop")));
            Assert.IsFalse(_session.ModeIsNav2);
        }

        [Test]
        public void Handover_GoalThenStick_CancelsAction()
        {
            var kitchen = _session.Waypoints.First(w => w.Name == "kitchen");
            Assert.IsTrue(_session.SendGoal(kitchen));
            string goalJson = _transport.Raw.Find(j => j.Contains("send_action_goal"));
            var goal = JObject.Parse(goalJson);
            Assert.AreEqual("navigate_to_pose", goal["action"].ToString());
            Assert.AreEqual("nav2_msgs/action/NavigateToPose", goal["action_type"].ToString());
            Assert.AreEqual(-1.9, goal["args"]["pose"]["pose"]["position"]["x"].Value<double>(), 1e-6);
            Assert.AreEqual("map", goal["args"]["pose"]["header"]["frame_id"].ToString());

            var mode = new ControlModeMachine();
            mode.Reset("nav2", RobotDeployment.Real);
            NavStickHandover.Apply(mode, _session, _link, "/control_mode", true, 0.02f);

            Assert.IsFalse(_session.GoalActive);
            Assert.IsTrue(_link.Published.Exists(p => p.topic == "/mission/cancel" && p.json.Contains("cancel")));
            Assert.IsTrue(_transport.Raw.Exists(j => j.Contains("cancel_action_goal") && j.Contains(goal["id"].ToString())));
            Assert.IsTrue(_link.Published.Exists(p => p.topic == "/control_mode" && p.json.Contains("teleop")));
        }

        [Test]
        public void Estop_CancelsNavGoal_BlocksAnother_AndDoesNotPublishNav2()
        {
            var kitchen = _session.Waypoints.First(w => w.Name == "kitchen");
            var scheme = ControlSchemeFactory.CreateDriveScheme(DriveProfile(RobotDeployment.Sim));
            var driving = DrivingStick();
            scheme.Tick(ref driving, _link);
            Assert.IsTrue(_session.SendGoal(kitchen));

            var stop = new TeleopInput
            {
                ButtonA = true,
                ButtonB = true,
                EmergencyStopPressed = true,
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.05f,
            };
            scheme.Tick(ref stop, _link);
            var drive = (DriveSchemeBase)scheme;
            Assert.IsTrue(drive.IsEmergencyStopActive);
            Assert.IsFalse(_session.GoalActive);
            Assert.IsFalse(_session.ModeIsNav2);
            Assert.IsTrue(_link.Published.Exists(p => p.topic == "/mission/cancel" && p.json.Contains("cancel")));
            Assert.IsTrue(_transport.Raw.Exists(j => j.Contains("cancel_action_goal")));
            Assert.IsFalse(_session.SendGoal(kitchen));

            _link.Published.Clear();
            var idle = new TeleopInput { LeftGrip = true, DeltaTime = 0.6f };
            scheme.Tick(ref idle, _link);
            Assert.IsFalse(PublishedNav2(), "Idle while e-stop is latched must not publish nav2");
            scheme.Stop(_link);
            Assert.IsFalse(PublishedNav2(), "Stop while e-stop is latched must not publish nav2");

            var reset = new TeleopInput { ResetEmergencyStop = true, LeftGrip = false, DeltaTime = 0.05f };
            scheme.Tick(ref reset, _link);
            var repress = new TeleopInput
            {
                LeftGrip = true,
                LeftStick = new Vector2(0f, 1f),
                DeltaTime = 0.05f,
            };
            scheme.Tick(ref repress, _link);
            Assert.IsFalse(drive.IsEmergencyStopActive);
            Assert.IsTrue(_session.SendGoal(kitchen));
        }

        [Test]
        public void OperateEStop_BlocksNavGoalUntilReset()
        {
            var kitchen = _session.Waypoints.First(w => w.Name == "kitchen");
            Assert.IsTrue(_session.SendGoal(kitchen));
            OperateSafetyLatch.Latch();
            Assert.IsFalse(_session.SendGoal(kitchen));
            int rawBefore = _transport.Raw.Count;
            _session.RequestMode("nav2");
            Assert.AreEqual(rawBefore, _transport.Raw.Count, "nav2 mode is refused while the operate latch is held");
            Assert.AreEqual("e-stop", _session.StatusText);

            var scheme = ControlSchemeFactory.CreateDriveScheme(DriveProfile(RobotDeployment.Real));
            Assert.IsFalse(_session.GoalActive, "Configuring a scheme under the operate latch cancels the goal");
            Assert.IsTrue(
                _transport.Published.Exists(p => p.topic == "/mission/cancel" && p.json.Contains("cancel"))
                || _link.Published.Exists(p => p.topic == "/mission/cancel" && p.json.Contains("cancel")));
            var input = DrivingStick();
            scheme.Tick(ref input, _link);
            Assert.IsFalse(_session.GoalActive);
            Assert.IsFalse(PublishedNav2(), "Operate e-stop must not publish nav2");

            OperateSafetyLatch.Clear();
            ((DriveSchemeBase)scheme).ReleaseUiStop(_link, gripHeld: false);
            Assert.IsTrue(_session.SendGoal(kitchen));
        }

        [Test]
        public void RobotLatchedEStop_CancelsNavGoalUntilSafetyClears()
        {
            var kitchen = _session.Waypoints.First(w => w.Name == "kitchen");
            Assert.IsTrue(_session.SendGoal(kitchen));
            var safety = new TeleopSafetySession(() => 10d);
            var scheme = ControlSchemeFactory.CreateDriveScheme(DriveProfile(RobotDeployment.Sim));
            safety.ApplySafetyState("{\"armed\":false,\"latched\":true,\"reason\":\"estop\"}");
            var input = DrivingStick();
            safety.Tick(_link, ref input, scheme, null, null);
            Assert.IsTrue(safety.Latched);
            Assert.IsFalse(_session.GoalActive);
            Assert.IsFalse(_session.SendGoal(kitchen));

            _link.Published.Clear();
            var idle = new TeleopInput { LeftGrip = true, DeltaTime = 0.6f };
            safety.Tick(_link, ref idle, scheme, null, null);
            Assert.IsFalse(PublishedNav2(), "Robot e-stop must not fall through to nav2");

            safety.ApplySafetyState("{\"armed\":true,\"latched\":false,\"reason\":\"\"}");
            safety.Tick(_link, ref input, scheme, null, null);
            Assert.IsFalse(safety.Latched);
            Assert.IsTrue(_session.SendGoal(kitchen));
        }

        // ── Waypoints ────────────────────────────────────────────────────────

        [Test]
        public void Contract_KitchenPose_MatchesNavigationYaml()
        {
            var kitchen = WaypointCatalog.ApartmentContract().First(w => w.Name == "kitchen");
            Assert.AreEqual(-1.9, kitchen.X, 1e-9);
            Assert.AreEqual(0.7, kitchen.Y, 1e-9);
            Assert.AreEqual(1.0, kitchen.Qz, 1e-9);
            Assert.AreEqual(0.0, kitchen.Qw, 1e-9);
            Assert.AreEqual("map", kitchen.FrameId);
        }

        [Test]
        public void Catalog_YamlKitchen_MatchesContract()
        {
            const string yaml = @"
frame_id: map
waypoints:
  - name: ""kitchen""
    yaw_deg: 180.0
    position:
      x: -1.9
      y: 0.7
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
";
            var parsed = WaypointCatalog.Parse(yaml);
            Assert.AreEqual(1, parsed.Count);
            var contract = WaypointCatalog.ApartmentContract().First(w => w.Name == "kitchen");
            Assert.AreEqual(contract.X, parsed[0].X, 1e-6);
            Assert.AreEqual(contract.Y, parsed[0].Y, 1e-6);
            Assert.AreEqual(contract.Qz, parsed[0].Qz, 1e-6);
            Assert.AreEqual(contract.Qw, parsed[0].Qw, 1e-6);
        }

        [Test]
        public void Catalog_JsonLocations_KitchenYaw()
        {
            const string json = "{\"frame_id\":\"map\",\"locations\":{\"kitchen\":{\"x\":-1.9,\"y\":0.7,\"yaw\":3.141592653589793}}}";
            var parsed = WaypointCatalog.Parse(json);
            Assert.AreEqual("kitchen", parsed[0].Name);
            Assert.AreEqual(-1.9, parsed[0].X, 1e-6);
            Assert.AreEqual(1.0, parsed[0].Qz, 1e-5);
            Assert.AreEqual(0.0, parsed[0].Qw, 1e-5);
        }

        [Test]
        public void Catalog_LocationYaml()
        {
            const string yaml = "locations:\n  desk:\n    x: 2.55\n    y: -0.45\n    yaw: 1.5707963267948966\n";
            var parsed = WaypointCatalog.Parse(yaml);
            Assert.AreEqual(1, parsed.Count);
            Assert.AreEqual("desk", parsed[0].Name);
            Assert.AreEqual(2.55, parsed[0].X, 1e-6);
            Assert.AreEqual(0.7071068, parsed[0].Qz, 1e-4);
        }

        [Test]
        public void Catalog_ServiceMessageWrapper()
        {
            var values = JObject.Parse("{\"success\":true,\"message\":\"{\\\"locations\\\":{\\\"door\\\":{\\\"x\\\":-0.3,\\\"y\\\":-1.45,\\\"yaw\\\":-1.5707963267948966}}}\"}");
            var parsed = WaypointCatalog.ParseServiceValues(values);
            Assert.AreEqual("door", parsed[0].Name);
            Assert.AreEqual(-0.3, parsed[0].X, 1e-6);
            Assert.AreEqual(-0.7071068, parsed[0].Qz, 1e-4);
        }

        [Test]
        public void Catalog_BadText_Empty()
        {
            Assert.AreEqual(0, WaypointCatalog.Parse("not a catalog").Count);
            Assert.AreEqual(0, WaypointCatalog.Parse("").Count);
        }

        [Test]
        public void ServiceResponse_ReplacesKitchen()
        {
            string payload = "{\"locations\":{\"kitchen\":{\"x\":1.25,\"y\":2,\"yaw\":0}}}";
            string frame = "{\"op\":\"service_response\",\"id\":\"" + NavMessages.ListServiceId
                + "\",\"result\":true,\"values\":{\"success\":true,\"message\":" + JsonString(payload) + "}}";
            _session.HandleProtocol(frame);
            var kitchen = _session.Waypoints.First(w => w.Name == "kitchen");
            Assert.AreEqual(1.25, kitchen.X, 1e-6);
            Assert.AreEqual("robot", _session.WaypointSource);
        }

        [Test]
        public void ServiceResponse_Garbage_KeepsContract()
        {
            _session.HandleProtocol("{\"op\":\"service_response\",\"id\":\"" + NavMessages.ListServiceId
                + "\",\"result\":true,\"values\":{\"success\":true,\"message\":\"nope\"}}");
            Assert.AreEqual(-1.9, _session.Waypoints.First(w => w.Name == "kitchen").X, 1e-6);
            Assert.AreEqual("apartment contract", _session.WaypointSource);
        }

        // ── Plan, feedback, map ──────────────────────────────────────────────

        [Test]
        public void Feedback_EtaAndDistance()
        {
            var kitchen = _session.Waypoints.First(w => w.Name == "kitchen");
            _session.SendGoal(kitchen);
            string id = JObject.Parse(_transport.Raw.Find(j => j.Contains("send_action_goal")))["id"].ToString();
            _session.HandleProtocol(
                "{\"op\":\"action_feedback\",\"id\":\"" + id + "\",\"action\":\"navigate_to_pose\",\"values\":{"
                + "\"estimated_time_remaining\":{\"sec\":12,\"nanosec\":500000000},"
                + "\"distance_remaining\":1.5}}");
            Assert.AreEqual(12.5f, _session.EtaSeconds.Value, 1e-4f);
            Assert.AreEqual(1.5f, _session.DistanceRemaining.Value, 1e-4f);
            Assert.AreEqual("executing", _session.StatusText);
            StringAssert.Contains("ETA 12.5 s", _session.HudLine);
            StringAssert.Contains("kitchen", _session.HudLine);
        }

        [Test]
        public void Result_Succeeded_ClearsGoal()
        {
            _session.SendGoal(_session.Waypoints.First(w => w.Name == "kitchen"));
            string id = JObject.Parse(_transport.Raw.Find(j => j.Contains("send_action_goal")))["id"].ToString();
            _session.HandleProtocol("{\"op\":\"action_result\",\"id\":\"" + id
                + "\",\"action\":\"navigate_to_pose\",\"status\":4,\"result\":true,\"values\":{}}");
            Assert.IsFalse(_session.GoalActive);
            Assert.AreEqual("succeeded", _session.StatusText);
        }

        [Test]
        public void Plan_Length()
        {
            const string json = "{\"header\":{\"frame_id\":\"map\"},\"poses\":["
                + "{\"pose\":{\"position\":{\"x\":0,\"y\":0}}},"
                + "{\"pose\":{\"position\":{\"x\":3,\"y\":0}}},"
                + "{\"pose\":{\"position\":{\"x\":3,\"y\":4}}}]}";
            var plan = NavMessages.ParsePlan(json);
            Assert.AreEqual(3, plan.Points.Count);
            Assert.AreEqual(7f, plan.LengthMeters, 1e-4f);
            _session.HandlePlan(json);
            StringAssert.Contains("plan 3 poses 7.00 m", _session.HudLine);
        }

        [Test]
        public void MapClick_OriginCell()
        {
            var map = new OccupancyMap
            {
                Width = 4,
                Height = 2,
                Resolution = 0.05f,
                OriginX = -3.455f,
                OriginY = -2.746f,
                Cells = new int[8],
            };
            Assert.IsTrue(NavMessages.TryMapClick(0f, 0f, 40f, 20f, map, out var x, out var y));
            Assert.AreEqual(-3.455f + 0.025f, x, 1e-4f);
            Assert.AreEqual(-2.746f + 0.025f, y, 1e-4f);
            Assert.IsFalse(NavMessages.TryMapClick(-1f, 0f, 40f, 20f, map, out _, out _));
        }

        [Test]
        public void MapParse_RejectsHugeGrid()
        {
            Assert.IsNull(NavMessages.ParseMap("{\"info\":{\"width\":9000,\"height\":10,\"resolution\":0.05,\"origin\":{\"position\":{\"x\":0,\"y\":0}}},\"data\":[]}"));
        }

        [Test]
        public void GoalStatusNames()
        {
            Assert.AreEqual("succeeded", NavMessages.GoalStatusName(4));
            Assert.AreEqual("canceled", NavMessages.GoalStatusName(5));
            Assert.AreEqual("aborted", NavMessages.GoalStatusName(6));
            Assert.IsTrue(NavMessages.IsTerminalStatus(4));
            Assert.IsFalse(NavMessages.IsTerminalStatus(2));
        }

        [Test]
        public void Fragment_Reassembles()
        {
            const string full = "{\"op\":\"service_response\",\"id\":\"ohho-wp-list\",\"result\":true}";
            int mid = full.Length / 2;
            var buffer = new RosbridgeFragmentBuffer();
            Assert.IsNull(buffer.Push("frag", 0, 2, full.Substring(0, mid)));
            Assert.AreEqual(full, buffer.Push("frag", 1, 2, full.Substring(mid)));
        }

        [Test]
        public void WaypointButtons_RebuildOnlyWhenCatalogChanges()
        {
            var host = new GameObject("nav-waypoint-buttons");
            host.SetActive(false);
            try
            {
                var panel = host.AddComponent<NavGoalController>();
                panel.BuildUiForTests();
                Button[] buttons = WaypointButtons(host);
                Assert.AreEqual(4, buttons.Length);

                _session.HandlePlan(
                    "{\"header\":{\"frame_id\":\"map\"},\"poses\":["
                    + "{\"pose\":{\"position\":{\"x\":0,\"y\":0}}},"
                    + "{\"pose\":{\"position\":{\"x\":3,\"y\":4}}}]}");
                _session.HandleModeEcho("{\"data\":\"nav2\"}");
                Assert.IsTrue(_session.SendGoal(_session.Waypoints.First(w => w.Name == "kitchen")));
                string goalId = JObject.Parse(_transport.Raw.Find(j => j.Contains("send_action_goal")))["id"].ToString();
                _session.HandleProtocol(
                    "{\"op\":\"action_feedback\",\"id\":\"" + goalId + "\",\"action\":\"navigate_to_pose\",\"values\":{"
                    + "\"estimated_time_remaining\":{\"sec\":4,\"nanosec\":0},"
                    + "\"distance_remaining\":1.25}}");
                panel.RefreshForTests();

                var status = host.transform.Find("Nav2 Panel/Nav Status").GetComponent<TMP_Text>();
                StringAssert.Contains("ETA 4.0 s", status.text);
                StringAssert.Contains("kitchen", status.text);
                Assert.AreEqual("Waypoints: apartment contract",
                    host.transform.Find("Nav2 Panel/Nav Source").GetComponent<TMP_Text>().text);
                Button[] kept = WaypointButtons(host);
                Assert.AreEqual(buttons.Length, kept.Length);
                for (int i = 0; i < buttons.Length; i++)
                    Assert.AreSame(buttons[i], kept[i]);

                string payload = "{\"locations\":{\"dock\":{\"x\":1,\"y\":2,\"yaw\":0}}}";
                string frame = "{\"op\":\"service_response\",\"id\":\"" + NavMessages.ListServiceId
                    + "\",\"result\":true,\"values\":{\"success\":true,\"message\":" + JsonString(payload) + "}}";
                _session.HandleProtocol(frame);
                panel.RefreshForTests();

                Button[] rebuilt = WaypointButtons(host);
                Assert.AreEqual(1, rebuilt.Length);
                Assert.AreNotSame(buttons[0], rebuilt[0]);
                Assert.AreEqual("dock",
                    host.transform.Find("Nav2 Panel/Waypoint Buttons").GetChild(0)
                        .GetComponentInChildren<TMP_Text>().text);
                Assert.AreEqual("Waypoints: robot",
                    host.transform.Find("Nav2 Panel/Nav Source").GetComponent<TMP_Text>().text);
            }
            finally
            {
                Object.DestroyImmediate(host);
            }
        }

        [Test]
        public void Configure_SameRobot_KeepsNav_OtherRobot_ClearsIt()
        {
            var garageA = Garage("simulated", null);
            garageA.UserRobot.Id = "user-a";
            var garageB = Garage("active", "real");
            garageB.UserRobot.Id = "user-b";
            var profileA = RobotProfileFactory.FromGarageRobot(garageA);
            var profileB = RobotProfileFactory.FromGarageRobot(garageB);
            Assert.AreEqual("user-a", profileA.UserRobotId);
            Assert.AreEqual("user-b", profileB.UserRobotId);
            Assert.AreEqual(profileA.RobotId, profileB.RobotId);

            var scheme = ControlSchemeFactory.CreateDriveScheme(profileA);
            Assert.IsNotNull(scheme);
            const string map = "{\"info\":{\"width\":1,\"height\":1,\"resolution\":0.05,\"origin\":{\"position\":{\"x\":1,\"y\":2}}},\"data\":[0]}";
            _session.HandleMap(map);
            string payload = "{\"locations\":{\"lab\":{\"x\":4,\"y\":5,\"yaw\":0}}}";
            _session.HandleProtocol("{\"op\":\"service_response\",\"id\":\"" + NavMessages.ListServiceId
                + "\",\"result\":true,\"values\":{\"success\":true,\"message\":" + JsonString(payload) + "}}");
            Assert.IsTrue(_session.SendGoal(_session.Waypoints.First(w => w.Name == "lab")));
            string goalId = JObject.Parse(_transport.Raw.Find(j => j.Contains("send_action_goal")))["id"].ToString();
            _session.HandleModeEcho("{\"data\":\"nav2\"}");
            _session.HandlePlan(
                "{\"header\":{\"frame_id\":\"map\"},\"poses\":["
                + "{\"pose\":{\"position\":{\"x\":0,\"y\":0}}},"
                + "{\"pose\":{\"position\":{\"x\":1,\"y\":0}}}]}");

            scheme.Configure(profileA);
            Assert.IsTrue(_session.GoalActive);
            Assert.IsNotNull(_session.Map);
            Assert.IsNotNull(_session.Plan);
            Assert.AreEqual("nav2", _session.ObservedMode);
            Assert.AreEqual("robot", _session.WaypointSource);
            Assert.AreEqual("lab", _session.GoalName);
            Assert.AreEqual(RobotDeployment.Sim, _session.DeploymentKind);

            scheme.Configure(profileB);
            Assert.IsFalse(_session.GoalActive);
            Assert.IsNull(_session.Map);
            Assert.IsNull(_session.Plan);
            Assert.AreEqual("teleop", _session.ObservedMode);
            Assert.AreEqual("apartment contract", _session.WaypointSource);
            Assert.IsNull(_session.GoalName);
            Assert.AreEqual("idle", _session.StatusText);
            Assert.AreEqual(RobotDeployment.Real, _session.DeploymentKind);
            Assert.AreEqual(-1.9, _session.Waypoints.First(w => w.Name == "kitchen").X, 1e-6);
            _session.HandleProtocol(
                "{\"op\":\"action_feedback\",\"id\":\"" + goalId + "\",\"values\":{\"distance_remaining\":1.5}}");
            Assert.IsNull(_session.DistanceRemaining);
            Assert.IsFalse(_session.GoalActive);
        }

        [Test]
        public void StartTeleop_FixedBaseOtherRobot_ClearsNavSession()
        {
            bool blocked = PreflightGate.BlocksMotion;
            PreflightGate.Allow();
            var host = new GameObject("nav-fixed-base-switch");
            try
            {
                var first = DriveProfile(RobotDeployment.Sim);
                first.UserRobotId = "user-a";
                var scheme = ControlSchemeFactory.CreateDriveScheme(first);
                Assert.IsNotNull(scheme);
                _session.HandleMap(
                    "{\"info\":{\"width\":1,\"height\":1,\"resolution\":0.05,\"origin\":{\"position\":{\"x\":0,\"y\":0}}},\"data\":[0]}");
                Assert.IsTrue(_session.SendGoal(_session.Waypoints.First(w => w.Name == "kitchen")));
                _session.HandleModeEcho("{\"data\":\"nav2\"}");

                var teleop = host.AddComponent<TeleopController>();
                teleop.SetLink(_link);
                var second = DriveProfile(RobotDeployment.Real);
                second.UserRobotId = "user-b";
                second.Drive = DriveKind.FixedBase;
                second.BaseDof = 0;
                second.HasArm = false;
                teleop.StartTeleop(second);

                Assert.IsFalse(_session.GoalActive);
                Assert.IsNull(_session.Map);
                Assert.AreEqual("teleop", _session.ObservedMode);
                Assert.AreEqual("apartment contract", _session.WaypointSource);
                Assert.AreEqual(RobotDeployment.Real, _session.DeploymentKind);
            }
            finally
            {
                Object.DestroyImmediate(host);
                if (blocked) PreflightGate.Hold(SessionKind.Real);
            }
        }

        [Test]
        public void MapOverlay_RobotSwitch_ClearsGridAndDisablesTaps()
        {
            const string mapJson = "{\"info\":{\"width\":2,\"height\":2,\"resolution\":0.05,\"origin\":{\"position\":{\"x\":0,\"y\":0}}},\"data\":[0,0,0,100]}";
            const string planJson = "{\"header\":{\"frame_id\":\"map\"},\"poses\":["
                + "{\"pose\":{\"position\":{\"x\":0.025,\"y\":0.025}}},"
                + "{\"pose\":{\"position\":{\"x\":0.075,\"y\":0.025}}}]}";
            var host = new GameObject("nav-map-overlay");
            host.SetActive(false);
            try
            {
                var first = DriveProfile(RobotDeployment.Sim);
                first.UserRobotId = "user-a";
                var second = DriveProfile(RobotDeployment.Real);
                second.UserRobotId = "user-b";
                _session.BeginRobot(first);

                var panel = host.AddComponent<NavGoalController>();
                panel.BuildUiForTests();
                var mapImage = host.transform.Find("Nav2 Panel/Map Image").GetComponent<RawImage>();
                var planImage = mapImage.transform.Find("Plan Image").GetComponent<RawImage>();
                var label = host.transform.Find("Nav2 Panel/Map Label").GetComponent<TMP_Text>();
                Assert.AreEqual("Waiting for /map", label.text);
                Assert.IsNull(mapImage.texture);
                Assert.IsFalse(mapImage.raycastTarget);
                Assert.IsNull(planImage.texture);
                Assert.IsFalse(planImage.enabled);
                Assert.AreEqual(0, GoalSends());
                panel.OnMapClicked(10f, 5f, 40f, 20f);
                Assert.AreEqual(0, GoalSends());

                _session.HandleMap(mapJson);
                _session.HandlePlan(planJson);
                panel.RefreshForTests();

                var mapTex = (Texture2D)mapImage.texture;
                var planTex = (Texture2D)planImage.texture;
                Assert.IsNotNull(mapTex);
                Assert.AreEqual(2, mapTex.width);
                Assert.AreEqual(2, mapTex.height);
                Assert.Greater(mapTex.GetPixel(0, 0).r, 0.8f);
                Assert.Less(mapTex.GetPixel(1, 1).r, 0.2f);
                Assert.IsNotNull(planTex);
                Assert.Greater(planTex.GetPixel(0, 0).b, 0.5f);
                Assert.IsTrue(mapImage.raycastTarget);
                Assert.IsTrue(planImage.enabled);
                Assert.AreEqual("Tap the map to send a goal", label.text);

                _session.HandleModeEcho("{\"data\":\"nav2\"}");
                _session.HandlePlan(planJson);
                panel.RefreshForTests();
                Assert.AreSame(mapTex, mapImage.texture);
                Assert.AreSame(planTex, planImage.texture);
                Assert.IsTrue(mapImage.raycastTarget);

                _session.BeginRobot(first);
                panel.RefreshForTests();
                Assert.AreSame(mapTex, mapImage.texture);
                Assert.AreSame(planTex, planImage.texture);
                Assert.IsTrue(mapImage.raycastTarget);
                Assert.AreEqual("Tap the map to send a goal", label.text);

                _session.BeginRobot(second);
                panel.RefreshForTests();
                Assert.IsNull(_session.Map);
                Assert.IsNull(_session.Plan);
                Assert.IsNull(mapImage.texture);
                Assert.IsFalse(mapImage.raycastTarget);
                Assert.IsNull(planImage.texture);
                Assert.IsFalse(planImage.enabled);
                Assert.AreEqual("Waiting for /map", label.text);
                Assert.AreEqual("Real robots stay in teleop when idle. Nav2 starts from this button or a goal.",
                    host.transform.Find("Nav2 Panel/Nav Hint").GetComponent<TMP_Text>().text);
                panel.OnMapClicked(10f, 5f, 40f, 20f);
                Assert.AreEqual(0, GoalSends());
                Assert.IsFalse(_session.GoalActive);

                _session.HandleMap(mapJson);
                panel.RefreshForTests();
                Assert.IsNotNull(mapImage.texture);
                Assert.AreNotSame(mapTex, mapImage.texture);
                Assert.IsTrue(mapImage.raycastTarget);
                Assert.IsTrue(planImage.enabled);
                Assert.AreEqual("Tap the map to send a goal", label.text);
                panel.OnMapClicked(10f, 5f, 40f, 20f);
                Assert.AreEqual(1, GoalSends());
                Assert.IsTrue(_session.GoalActive);
                Assert.AreEqual("map", _session.GoalName);
            }
            finally
            {
                Object.DestroyImmediate(host);
            }
        }

        [Test]
        public void RequestWaypointList_CallsParamAndService()
        {
            _session.RequestWaypointList();
            Assert.IsTrue(_transport.Raw.Exists(j => j.Contains("/ohho/nav/list_waypoints")));
            Assert.IsTrue(_transport.Raw.Exists(j => j.Contains("/ohho/waypoints")));
            Assert.IsTrue(_transport.Raw.Exists(j => j.Contains("\"/waypoints\"")));
        }

        // ── Profile kind ─────────────────────────────────────────────────────

        [Test]
        public void Factory_SimulatedRobot_IsSim()
        {
            var profile = RobotProfileFactory.FromGarageRobot(Garage("simulated", null));
            Assert.AreEqual(RobotDeployment.Sim, profile.DeploymentKind);
        }

        [Test]
        public void Factory_KindReal_OverridesSimulatedStatus()
        {
            var profile = RobotProfileFactory.FromGarageRobot(Garage("simulated", "real"));
            Assert.AreEqual(RobotDeployment.Real, profile.DeploymentKind);
        }

        /// <summary>
        /// Forward stick with the deadman held and e-stop clear. A bare stick
        /// never enters teleop while the deadman defaults on.
        /// </summary>
        private static TeleopInput DrivingStick()
        {
            return new TeleopInput
            {
                LeftStick = new UnityEngine.Vector2(0f, 1f),
                LeftGrip = true,
                RightGrip = false,
                BothGrips = false,
                BothGripsPressed = false,
                ResetEmergencyStop = false,
                DeltaTime = 0.05f,
            };
        }

        private static RobotProfile DriveProfile(string deployment)
        {
            return new RobotProfile
            {
                RobotId = "test",
                Name = "Test",
                Drive = DriveKind.Mecanum,
                DeploymentKind = deployment,
                MaxLinVel = 0.5f,
                MaxAngVel = 1f,
                Topics = new RobotTopics { CmdVel = "/cmd_vel/teleop", ControlMode = "/control_mode" },
            };
        }

        private static GarageRobot Garage(string status, string kind)
        {
            var config = new Dictionary<string, object>();
            if (kind != null) config["kind"] = kind;
            return new GarageRobot
            {
                UserRobot = new UserRobot { Status = status, Config = config, Name = "Bot" },
                HardwareModel = new VrHardwareModel
                {
                    Id = "omnibot",
                    Name = "OmniBot",
                    Locomotion = "mecanum",
                    HasArm = true,
                },
                RobotType = new VrRobotType { Id = "omnibot", Name = "OmniBot" },
                Category = new VrCategory { Id = "mobile-manipulator" },
            };
        }

        private bool PublishedNav2()
        {
            return _link.Published.Exists(p => p.topic == "/control_mode" && p.json.Contains("\"nav2\""));
        }

        private int GoalSends()
        {
            return _transport.Raw.Count(j => j.Contains("send_action_goal"));
        }

        private static Button[] WaypointButtons(GameObject host)
        {
            var root = host.transform.Find("Nav2 Panel/Waypoint Buttons");
            var buttons = new Button[root.childCount];
            for (int i = 0; i < root.childCount; i++)
                buttons[i] = root.GetChild(i).GetComponent<Button>();
            return buttons;
        }

        private static string JsonString(string value)
        {
            return "\"" + value.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
        }

        private sealed class FakeLink : IRobotLink
        {
            public readonly List<(string topic, string json)> Published = new List<(string topic, string json)>();
            public bool IsConnected => true;
            public void Advertise(string topic, string type) { }
            public void Publish<T>(string topic, T msg) =>
                Published.Add((topic, Newtonsoft.Json.JsonConvert.SerializeObject(msg)));
            public void Subscribe(string topic, string type, System.Action<string> callback) { }
            public void Unsubscribe(string topic) { }
            public void CallService(string service, string type, System.Action<bool, string> onResponse) { }
        }

        private sealed class FakeTransport : INavTransport
        {
            public readonly List<string> Raw = new List<string>();
            public readonly List<(string topic, string json)> Published = new List<(string topic, string json)>();
            public bool IsConnected => true;
            public void SendRaw(string json) => Raw.Add(json);
            public void Publish<T>(string topic, T message) =>
                Published.Add((topic, Newtonsoft.Json.JsonConvert.SerializeObject(message)));
            public void Advertise(string topic, string type) { }
            public void Subscribe(string topic, string type, System.Action<string> callback) { }
        }
    }
}
