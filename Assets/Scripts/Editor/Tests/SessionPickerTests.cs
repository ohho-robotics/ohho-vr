using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using OmniBot.VR.App;
using OmniBot.VR.Control;
using OmniBot.VR.Control.Drive;
using OmniBot.VR.Control.Preflight;
using OmniBot.VR.Control.Safety;
using OmniBot.VR.Control.Session;
using OmniBot.VR.Core;
using OmniBot.VR.Core.Platform;
using OmniBot.VR.Nav;
using OmniBot.VR.Policy;

namespace OmniBot.VR.Tests
{
    /// <summary>
    /// Garage → Simulate / Drive real → target. Manual IP works with an empty
    /// registry. Real launches stay blocked until the safety notice is
    /// acknowledged, and the deadman cannot be switched off.
    /// </summary>
    public class SessionPickerTests
    {
        private const string RobotA = "ohh-212-test-robot-a";
        private const string RobotB = "ohh-212-test-robot-b";
        private const string TutorialKey = "ohho.real_deadman_tutorial_seen";

        private string _lastId;
        private string _lastName;
        private string _lastIp;
        private string _lastKind;
        private string _lastAlias;
        private int _lastPort;
        private int _lastHas;
        private int _tutorial;

        [SetUp]
        public void SetUp()
        {
            _lastId = PlayerPrefs.GetString("ohho.last_robot_id", "");
            _lastName = PlayerPrefs.GetString("ohho.last_robot_name", "");
            _lastIp = PlayerPrefs.GetString("ohho.last_robot_ip", "");
            _lastKind = PlayerPrefs.GetString("ohho.last_session_kind", "");
            _lastAlias = PlayerPrefs.GetString("ohho.last_robot_alias", "");
            _lastPort = PlayerPrefs.GetInt("ohho.last_robot_port", 9090);
            _lastHas = PlayerPrefs.GetInt("ohho.has_last_robot", 0);
            _tutorial = PlayerPrefs.GetInt(TutorialKey, 0);
            Isolate();
        }

        [TearDown]
        public void TearDown()
        {
            Isolate();
            PlayerPrefs.SetString("ohho.last_robot_id", _lastId);
            PlayerPrefs.SetString("ohho.last_robot_name", _lastName);
            PlayerPrefs.SetString("ohho.last_robot_ip", _lastIp);
            PlayerPrefs.SetInt("ohho.last_robot_port", _lastPort);
            PlayerPrefs.SetInt("ohho.has_last_robot", _lastHas);
            if (string.IsNullOrEmpty(_lastKind)) PlayerPrefs.DeleteKey("ohho.last_session_kind");
            else PlayerPrefs.SetString("ohho.last_session_kind", _lastKind);
            if (string.IsNullOrEmpty(_lastAlias)) PlayerPrefs.DeleteKey("ohho.last_robot_alias");
            else PlayerPrefs.SetString("ohho.last_robot_alias", _lastAlias);
            if (_tutorial == 0) PlayerPrefs.DeleteKey(TutorialKey);
            else PlayerPrefs.SetInt(TutorialKey, _tutorial);
            PlayerPrefs.Save();
        }

        private static void Isolate()
        {
            LastRobotStore.ClearChoice(RobotA);
            LastRobotStore.ClearChoice(RobotB);
            RealRobotSafety.ResetTutorial();
            SessionCatalog.ResetSharedForTests();
            ActiveSession.Clear();
        }

        [Test]
        public void ThreeClicks_SimulateLocal_UsesPrefilledManualIp()
        {
            // Garage is already open. Click 1 is the robot card, click 2 is Simulate,
            // click 3 is Local desktop — that click is this call. No registry, no extra confirm.
            var plan = SessionPlanner.LocalDesktop("omnibot", null, "192.168.1.101", 9090, new SessionCatalog(), null);

            Assert.IsTrue(plan.ReadyToDrive);
            Assert.IsTrue(plan.DeadmanRequired);
            Assert.AreEqual(SessionKind.Sim, plan.Endpoint.Kind);
            Assert.AreEqual(SessionTarget.LocalDesktop, plan.Endpoint.Target);
            Assert.AreEqual("ws://192.168.1.101:9090", plan.Endpoint.RosbridgeUrl);
            Assert.IsFalse(plan.Endpoint.Offline);
            Assert.IsFalse(plan.Endpoint.RosbridgeUrl.Contains("token"));
        }

        [Test]
        public void LocalDesktop_ListedSession_UsesThatEndpoint()
        {
            var catalog = new SessionCatalog();
            catalog.Add(new RobotEndpoint
            {
                RobotId = "omnibot",
                SessionId = "desk-1",
                Kind = SessionKind.Sim,
                Target = SessionTarget.LocalDesktop,
                RosbridgeUrl = "ws://10.0.0.5:9090",
                Host = "10.0.0.5",
                Port = 9090,
                VideoUrls = new List<string> { "http://10.0.0.5:8080/stream?topic=/camera/front/image_raw" },
                WebrtcSignalUrl = "ws://10.0.0.5:8443",
                Label = "Lab desktop",
            });

            var plan = SessionPlanner.LocalDesktop("omnibot", null, "192.168.1.101", 9090, catalog, "desk-1");

            Assert.IsTrue(plan.ReadyToDrive);
            Assert.AreEqual("ws://10.0.0.5:9090", plan.Endpoint.RosbridgeUrl);
            Assert.AreEqual("ws://10.0.0.5:8443", plan.Endpoint.WebrtcSignalUrl);
            Assert.AreEqual(1, plan.Endpoint.VideoUrls.Count);
        }

        [Test]
        public void LocalDesktop_UnknownSession_IsBlocked()
        {
            var plan = SessionPlanner.LocalDesktop("omnibot", null, "192.168.1.101", 9090, new SessionCatalog(), "missing");
            Assert.IsFalse(plan.ReadyToDrive);
            Assert.IsNull(plan.Endpoint);
        }

        [Test]
        public void ManualIp_RejectsBadOctetAndPort()
        {
            Assert.IsFalse(EndpointAddresses.TryManual("omnibot", "999.1.1.1", 9090,
                SessionKind.Sim, SessionTarget.LocalDesktop, out _, out var octetError));
            StringAssert.Contains("0–255", octetError);

            Assert.IsFalse(EndpointAddresses.TryManual("omnibot", "192.168.1.101", 0,
                SessionKind.Sim, SessionTarget.LocalDesktop, out _, out var portError));
            StringAssert.Contains("1 and 65535", portError);
        }

        [Test]
        public void ManualIp_HostAndPortInOneField_OverridesPortArgument()
        {
            Assert.IsTrue(EndpointAddresses.TryManual("omnibot", "192.168.1.20:9091", 9090,
                SessionKind.Sim, SessionTarget.LocalDesktop, out var endpoint, out _));
            Assert.AreEqual("ws://192.168.1.20:9091", endpoint.RosbridgeUrl);
            Assert.AreEqual(9091, endpoint.Port);
        }

        [Test]
        public void CloudUrl_KeepsWss()
        {
            var plan = SessionPlanner.CloudAttach("omnibot", "wss://gpu.example.com/rosbridge");
            Assert.IsTrue(plan.ReadyToDrive);
            Assert.AreEqual(SessionKind.Sim, plan.Endpoint.Kind);
            Assert.AreEqual(SessionTarget.Cloud, plan.Endpoint.Target);
            Assert.AreEqual("wss://gpu.example.com/rosbridge", plan.Endpoint.RosbridgeUrl);
            Assert.AreEqual("gpu.example.com", plan.Endpoint.Host);
            Assert.AreEqual(443, plan.Endpoint.Port);
        }

        [Test]
        public void InHeadset_IsOfflineSim_WithNoSocket()
        {
            var plan = SessionPlanner.InHeadset("omnibot");
            Assert.IsTrue(plan.ReadyToDrive);
            Assert.IsTrue(plan.Endpoint.Offline);
            Assert.AreEqual(SessionKind.Sim, plan.Endpoint.Kind);
            Assert.AreEqual(SessionTarget.InHeadset, plan.Endpoint.Target);
            Assert.IsTrue(string.IsNullOrEmpty(plan.Endpoint.RosbridgeUrl));

            var link = new OfflineKinematicLink();
            Assert.IsTrue(link.IsConnected);
            link.Publish("/cmd_vel/teleop", new TwistMsg(new Vector3Msg(0.1f, 0f, 0f), new Vector3Msg(0f, 0f, 0f)));
            Assert.AreEqual(0.1f, link.LastTwist.linear.x, 0.0001f);

            bool called = false;
            bool ok = true;
            string values = "unset";
            link.CallService(OhhoSafetyContract.EstopResetService, OhhoSafetyContract.EstopResetType, (success, payload) =>
            {
                called = true;
                ok = success;
                values = payload;
            });
            Assert.IsTrue(called);
            Assert.IsFalse(ok);
            Assert.AreEqual("", values);
        }

        [Test]
        public void CloudStart_WithoutOrchestrator_DoesNotInventAnEndpoint()
        {
            var plan = SessionPlanner.CloudStart("omnibot", new SessionCatalog());
            Assert.IsFalse(plan.ReadyToDrive);
            Assert.IsNull(plan.Endpoint);
            StringAssert.Contains("Local desktop", plan.BlockReason);
        }

        [Test]
        public void CloudStart_RealTarget_IsRefused()
        {
            var catalog = new SessionCatalog();
            catalog.CloudStarter = _ => new RobotEndpoint
            {
                Kind = SessionKind.Real,
                RosbridgeUrl = "ws://10.0.0.2:9090",
            };
            var plan = SessionPlanner.CloudStart("omnibot", catalog);
            Assert.IsFalse(plan.ReadyToDrive);
            Assert.IsNull(plan.Endpoint);
            StringAssert.Contains("Drive real", plan.BlockReason);
        }

        [Test]
        public void CloudStart_WithOrchestrator_ReturnsSimEndpoint()
        {
            var catalog = new SessionCatalog();
            catalog.CloudStarter = _ => new RobotEndpoint
            {
                Kind = SessionKind.Sim,
                RosbridgeUrl = "wss://cloud.example/ros",
                Host = "cloud.example",
                Port = 443,
            };
            var plan = SessionPlanner.CloudStart("omnibot", catalog);
            Assert.IsTrue(plan.ReadyToDrive);
            Assert.AreEqual(SessionKind.Sim, plan.Endpoint.Kind);
            Assert.AreEqual(SessionTarget.Cloud, plan.Endpoint.Target);
            Assert.AreEqual("wss://cloud.example/ros", plan.Endpoint.RosbridgeUrl);
        }

        [Test]
        public void DriveReal_RequiresSafetyEveryTime_AndTutorialOnlyOnce()
        {
            var blocked = SessionPlanner.DriveReal("omnibot", null, "192.168.1.101", 9090,
                null, null, safetyAcknowledged: false, tutorialSeen: false);
            Assert.IsFalse(blocked.ReadyToDrive);
            Assert.IsTrue(blocked.ShowDeadmanTutorial);
            Assert.IsTrue(blocked.DeadmanRequired);
            Assert.AreEqual(RealRobotSafety.SafetyBlock, blocked.BlockReason);

            var first = SessionPlanner.DriveReal("omnibot", null, "192.168.1.101", 9090,
                null, null, safetyAcknowledged: true, tutorialSeen: false);
            Assert.IsTrue(first.ReadyToDrive);
            Assert.IsTrue(first.ShowDeadmanTutorial);
            Assert.AreEqual(SessionKind.Real, first.Endpoint.Kind);
            Assert.IsFalse(first.Endpoint.Offline);

            var again = SessionPlanner.DriveReal("omnibot", null, "192.168.1.101", 9090,
                null, null, safetyAcknowledged: false, tutorialSeen: true);
            Assert.IsFalse(again.ReadyToDrive);
            Assert.IsFalse(again.ShowDeadmanTutorial);
        }

        [Test]
        public void SavedRealChoice_StillRequiresAcknowledgement()
        {
            Assert.IsTrue(EndpointAddresses.TryManual("omnibot", "10.0.0.8", 9090,
                SessionKind.Real, SessionTarget.ManualIp, out var saved, out _));
            var skipped = SessionPlanner.FromSaved(saved, safetyAcknowledged: false, tutorialSeen: true);
            Assert.IsFalse(skipped.ReadyToDrive);
            Assert.AreEqual(SessionKind.Real, skipped.Endpoint.Kind);

            var sim = SessionPlanner.FromSaved(
                EndpointAddresses.InHeadset("omnibot"), safetyAcknowledged: false, tutorialSeen: true);
            Assert.IsTrue(sim.ReadyToDrive);
            Assert.AreEqual(SessionKind.Sim, sim.Endpoint.Kind);
        }

        [Test]
        public void ApplyActiveSession_KeepsSimKind_TurboAndDeadmanOptOut()
        {
            var rebuilt = new RobotProfile
            {
                RobotId = "omnibot",
                Name = "OmniBot",
                Drive = DriveKind.Mecanum,
                Holonomic = true,
                MaxLinVel = 0.2f,
                MaxLatVel = 0.2f,
                MaxAngVel = 1f,
                Topics = new RobotTopics
                {
                    CmdVel = "/cmd_vel/teleop",
                    ControlMode = "/control_mode",
                    EmergencyStop = "/emergency_stop",
                },
            };
            Assert.AreEqual(SessionKind.Real, rebuilt.Kind, "A descriptor profile defaults to real");
            rebuilt.DisableDeadmanForSimulation();
            Assert.IsTrue(rebuilt.RequireDeadman, "A real default ignores the sim deadman opt-out");

            ActiveSession.Set(new RobotEndpoint { RobotId = "omnibot", Kind = SessionKind.Sim });
            try
            {
                RealRobotSafety.ApplyActiveSession(rebuilt);
                Assert.AreEqual(SessionKind.Sim, rebuilt.Kind);
                Assert.IsTrue(rebuilt.RequireDeadman, "Applying the session keeps the deadman on");
                rebuilt.DisableDeadmanForSimulation();
                Assert.IsFalse(rebuilt.RequireDeadman, "A sim session can still opt out of the deadman");
                rebuilt.RequireDeadman = true;

                var link = new FakeRobotLink();
                var scheme = new MecanumDriveScheme();
                scheme.Configure(rebuilt);
                var half = new TeleopInput
                {
                    LeftGrip = true,
                    LeftStick = new Vector2(0f, 0.5f),
                    DeltaTime = 0.1f,
                };
                scheme.Tick(ref half, link);
                float baseline = ((TwistMsg)link.Published.Find(p => p.Topic == "/cmd_vel/teleop").Msg).linear.x;
                link.Published.Clear();
                var turbo = half;
                turbo.RightTrigger = true;
                scheme.Tick(ref turbo, link);
                float boosted = ((TwistMsg)link.Published.Find(p => p.Topic == "/cmd_vel/teleop").Msg).linear.x;
                Assert.Greater(boosted, baseline, "A rebuilt sim profile still accepts turbo");
                Assert.LessOrEqual(boosted, 0.2f);

                rebuilt.Kind = SessionKind.Real;
                scheme.Configure(rebuilt);
                link.Published.Clear();
                scheme.Tick(ref turbo, link);
                float real = ((TwistMsg)link.Published.Find(p => p.Topic == "/cmd_vel/teleop").Msg).linear.x;
                Assert.AreEqual(baseline, real, 0.0001f, "The real default ignores turbo");
            }
            finally
            {
                ActiveSession.Clear();
            }
        }

        [Test]
        public void DescriptorRebuild_CopiesActiveSessionKindOntoTheProfile()
        {
            ActiveSession.Set(new RobotEndpoint { RobotId = "omnibot-pro", Kind = SessionKind.Sim });
            PreflightGate.Allow();
            var appGo = new GameObject("descriptor-rebuild-app");
            var teleopGo = new GameObject("descriptor-rebuild-teleop");
            PolicyPanelController[] panelsBefore = UnityEngine.Object.FindObjectsOfType<PolicyPanelController>();
            try
            {
                var teleop = teleopGo.AddComponent<TeleopController>();
                teleop.SetLink(new FakeRobotLink());
                var app = appGo.AddComponent<OhhoVrApp>();
                var flags = BindingFlags.Instance | BindingFlags.NonPublic;
                typeof(OhhoVrApp).GetField("teleopController", flags).SetValue(app, teleop);
                typeof(OhhoVrApp).GetField("_activeRobot", flags).SetValue(app, new GarageRobot
                {
                    RobotType = new VrRobotType { Id = "mobile" },
                    Category = new VrCategory { Id = "mobile-manipulator" },
                    HardwareModel = new VrHardwareModel
                    {
                        Id = "omnibot-pro",
                        Name = "OmniBot Pro",
                        Locomotion = "mecanum",
                        HasArm = false,
                    },
                });

                var apply = typeof(OhhoVrApp).GetMethod("ApplyActiveProfile", flags);
                Assert.IsTrue((bool)apply.Invoke(app, new object[] { false }));
                Assert.IsNotNull(teleop.Profile);
                Assert.AreEqual(SessionKind.Sim, teleop.Profile.Kind,
                    "A descriptor rebuild during a sim session keeps kind=sim");
                Assert.IsTrue(teleop.Profile.RequireDeadman);
                teleop.StopTeleop();
            }
            finally
            {
                ActiveSession.Clear();
                NavGoalSession.Active.ResetForTests();
                OperateSafetyLatch.Clear();
                PolicyPanelController[] panelsAfter = UnityEngine.Object.FindObjectsOfType<PolicyPanelController>();
                for (int i = 0; i < panelsAfter.Length; i++)
                {
                    bool existed = false;
                    for (int j = 0; j < panelsBefore.Length; j++)
                    {
                        if (panelsAfter[i] == panelsBefore[j]) existed = true;
                    }
                    if (!existed && panelsAfter[i] != null)
                        UnityEngine.Object.DestroyImmediate(panelsAfter[i].gameObject);
                }
                UnityEngine.Object.DestroyImmediate(teleopGo);
                UnityEngine.Object.DestroyImmediate(appGo);
            }
        }

        [Test]
        public void RealLaunch_ForcesDeadmanBackOn()
        {
            var profile = new RobotProfile();
            Assert.IsTrue(profile.RequireDeadman);
            profile.Kind = SessionKind.Sim;
            profile.DisableDeadmanForSimulation();
            Assert.IsFalse(profile.RequireDeadman);

            RealRobotSafety.Apply(profile, SessionKind.Real);
            Assert.IsTrue(profile.RequireDeadman);
            Assert.IsFalse(RealRobotSafety.TryDisableDeadman());
        }

        [Test]
        public void SafetyCopy_SaysDeadmanCannotBeTurnedOff()
        {
            StringAssert.Contains("cannot be turned off", RealRobotSafety.Notice);
            StringAssert.Contains("will not turn it off", RealRobotSafety.Tutorial);
            StringAssert.DoesNotContain("optional", RealRobotSafety.Notice.ToLowerInvariant());
            StringAssert.DoesNotContain("optional", RealRobotSafety.Tutorial.ToLowerInvariant());
            Assert.IsFalse(RealRobotSafety.TutorialSeen);
            RealRobotSafety.MarkTutorialSeen();
            Assert.IsTrue(RealRobotSafety.TutorialSeen);
        }

        [Test]
        public void Banner_SimIsBlue_RealIsOrange()
        {
            Assert.AreEqual(SessionBannerStyle.SimBlue, SessionBannerStyle.ColorFor(SessionKind.Sim));
            Assert.AreEqual(SessionBannerStyle.RealOrange, SessionBannerStyle.ColorFor(SessionKind.Real));
            Assert.Greater(SessionBannerStyle.SimBlue.b, SessionBannerStyle.SimBlue.r);
            Assert.Greater(SessionBannerStyle.RealOrange.r, SessionBannerStyle.RealOrange.b);
            Assert.AreNotEqual(SessionBannerStyle.SimBlue, SessionBannerStyle.RealOrange);
            Assert.AreEqual("SIM", SessionBannerStyle.LabelFor(SessionKind.Sim));
            Assert.AreEqual("REAL", SessionBannerStyle.LabelFor(SessionKind.Real));
        }

        [Test]
        public void Choice_IsStoredPerRobot()
        {
            Assert.IsTrue(EndpointAddresses.TryManual(RobotA, "192.168.1.101", 9090,
                SessionKind.Sim, SessionTarget.LocalDesktop, out var sim, out _));
            Assert.IsTrue(EndpointAddresses.TryManual(RobotB, "10.0.0.8", 9090,
                SessionKind.Real, SessionTarget.ManualIp, out var real, out _));
            sim.VideoUrls.Add("http://192.168.1.101:8080/stream?topic=/camera/front/image_raw");

            LastRobotStore.Save(RobotA, "Omni A", sim.Host, sim.Port, sim);
            LastRobotStore.SaveChoice(RobotB, real);

            Assert.IsTrue(LastRobotStore.TryLoadChoice(RobotA, out var loadedA));
            Assert.IsTrue(LastRobotStore.TryLoadChoice(RobotB, out var loadedB));
            Assert.AreEqual(SessionKind.Sim, loadedA.Kind);
            Assert.AreEqual(SessionTarget.LocalDesktop, loadedA.Target);
            Assert.AreEqual("ws://192.168.1.101:9090", loadedA.RosbridgeUrl);
            Assert.AreEqual(1, loadedA.VideoUrls.Count);
            Assert.AreEqual(SessionKind.Real, loadedB.Kind);
            Assert.AreEqual("ws://10.0.0.8:9090", loadedB.RosbridgeUrl);
            Assert.AreEqual(SessionKind.Sim, LastRobotStore.LastSessionKind);

            LastRobotStore.Save(RobotB, "Omni B", real.Host, real.Port, real);
            Assert.AreEqual(SessionKind.Real, LastRobotStore.LastSessionKind);
            Assert.AreEqual(SessionKind.Sim, LastRobotStore.TryLoadChoice(RobotA, out var stillA) ? stillA.Kind : SessionKind.Real);
        }

        [Test]
        public void VideoUrls_UsePort8080_AndKeepTopicSlashes()
        {
            var urls = EndpointAddresses.VideoUrlsFor("192.168.1.101", new[] { "/camera/front/image_raw" });
            Assert.AreEqual(1, urls.Count);
            Assert.AreEqual("http://192.168.1.101:8080/stream?topic=/camera/front/image_raw", urls[0]);
        }

        [Test]
        public void RegistryRow_ParsesEndpoint_AndIgnoresDeadmanFalse()
        {
            const string json = "[" +
                "{\"id\":\"abc\",\"robot_id\":\"omnibot\",\"kind\":\"real\",\"target\":\"announced\"," +
                "\"rosbridge_url\":\"ws://10.0.0.8:9090\"," +
                "\"video_urls\":[\"http://10.0.0.8:8080/stream?topic=/camera/front/image_raw\"]," +
                "\"webrtc_signal_url\":\"ws://10.0.0.8:8443\",\"auth_token\":\"tok\",\"deadman\":false}" +
                "]";
            var rows = RobotSessionRows.Parse(json, out var error);
            Assert.IsNull(error);
            Assert.AreEqual(1, rows.Count);
            Assert.AreEqual(SessionKind.Real, rows[0].Kind);
            Assert.AreEqual("tok", rows[0].AuthToken);
            Assert.AreEqual("ws://10.0.0.8:9090", rows[0].RosbridgeUrl);
            Assert.IsFalse(rows[0].RosbridgeUrl.Contains("tok"));

            var plan = SessionPlanner.FromSaved(rows[0], safetyAcknowledged: true, tutorialSeen: true);
            Assert.IsTrue(plan.ReadyToDrive);
            Assert.IsTrue(plan.DeadmanRequired);
            Assert.AreEqual(SessionKind.Real, plan.Endpoint.Kind);
        }

        [Test]
        public void RegistryRow_CloudMarkedReal_StaysOnTheRealList()
        {
            const string json = "{\"rosbridge_url\":\"ws://10.1.1.1:9090\",\"kind\":\"real\",\"target\":\"cloud\"}";
            var rows = RobotSessionRows.Parse(json, out var error);
            Assert.IsNull(error);
            Assert.AreEqual(SessionKind.Real, rows[0].Kind);

            var catalog = new SessionCatalog();
            catalog.ReplaceRemote(rows);
            Assert.AreEqual(0, catalog.ListCloud("omnibot", null).Count);
            Assert.AreEqual(1, catalog.ListReal("omnibot", null).Count);
        }

        [Test]
        public void ReplaceRemote_KeepsManualRows()
        {
            var catalog = new SessionCatalog();
            catalog.Add(new RobotEndpoint
            {
                RobotId = "omnibot",
                SessionId = "manual",
                Kind = SessionKind.Sim,
                Target = SessionTarget.LocalDesktop,
                RosbridgeUrl = "ws://192.168.1.101:9090",
                Source = "manual",
            });
            var remote = RobotSessionRows.Parse(
                "[{\"id\":\"reg\",\"robot_id\":\"omnibot\",\"kind\":\"sim\",\"target\":\"local\",\"rosbridge_url\":\"ws://10.0.0.9:9090\"}]",
                out _);
            catalog.ReplaceRemote(remote);
            Assert.AreEqual(2, catalog.ListLocalSim("omnibot", null).Count);
            catalog.ReplaceRemote(new List<RobotEndpoint>());
            Assert.AreEqual(1, catalog.ListLocalSim("omnibot", null).Count);
            Assert.AreEqual("manual", catalog.ListLocalSim("omnibot", null)[0].SessionId);
        }

        [Test]
        public void Announcement_RequiresKind()
        {
            var missing = new Dictionary<string, string>
            {
                { "rosbridge", "ws://10.0.0.4:9090" },
            };
            Assert.IsFalse(RobotSessionRows.TryFromAnnouncement(missing, "omnibot", out _, out var error));
            StringAssert.Contains("kind", error);

            var sim = new Dictionary<string, string>
            {
                { "kind", "sim" },
                { "rosbridge", "ws://10.0.0.4:9090" },
                { "name", "desktop-sim" },
                { "video", "http://10.0.0.4:8080/stream?topic=/camera/front/image_raw" },
            };
            Assert.IsTrue(RobotSessionRows.TryFromAnnouncement(sim, "omnibot", out var endpoint, out _));
            Assert.AreEqual(SessionKind.Sim, endpoint.Kind);
            Assert.AreEqual("desktop-sim", endpoint.Label);
            Assert.AreEqual("announcement", endpoint.Source);
            Assert.AreEqual(1, endpoint.VideoUrls.Count);
        }

        [Test]
        public void UnknownKind_IsNotFiledAsSim()
        {
            const string json = "[{\"id\":\"x\",\"robot_id\":\"omnibot\",\"kind\":\"physical\",\"target\":\"local\",\"rosbridge_url\":\"ws://10.0.0.8:9090\"}]";
            var rows = RobotSessionRows.Parse(json, out var error);
            Assert.IsNull(error);
            Assert.AreEqual(0, rows.Count);

            var announced = new Dictionary<string, string>
            {
                { "kind", "physical" },
                { "target", "local" },
                { "rosbridge", "ws://10.0.0.8:9090" },
            };
            Assert.IsFalse(RobotSessionRows.TryFromAnnouncement(announced, "omnibot", out _, out var announceError));
            StringAssert.Contains("kind", announceError);

            const string offline = "[{\"id\":\"off\",\"robot_id\":\"omnibot\",\"kind\":\"offline\",\"target\":\"local\"}]";
            var offlineRows = RobotSessionRows.Parse(offline, out var offlineError);
            Assert.IsNull(offlineError);
            Assert.AreEqual(1, offlineRows.Count);
            Assert.IsTrue(offlineRows[0].Offline);
            Assert.AreEqual(SessionKind.Sim, offlineRows[0].Kind);
        }
    }
}
