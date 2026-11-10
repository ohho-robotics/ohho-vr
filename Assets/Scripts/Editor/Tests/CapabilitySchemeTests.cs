using System;
using System.IO;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using OmniBot.VR.Control;
using OmniBot.VR.Control.Drive;
using OmniBot.VR.Control.Manip;
using OmniBot.VR.Core;
using UnityEngine;
using LatchedRobotDescriptor = OmniBot.VR.Descriptor.RobotDescriptor;

namespace OmniBot.VR.Tests
{
    /// <summary>
    /// Drive and manipulation schemes come from descriptor capabilities.
    /// A catalog category or a robot id must not choose them.
    /// </summary>
    public class CapabilitySchemeTests
    {
        static readonly string[] BuiltInRobotIds =
        {
            "omnibot",
            "so101",
            "ur5e",
            "unitree",
            "figure-02",
            "px4",
            "crazyflie",
            "yahboom",
        };

        // Pre-existing files that pin catalog topics, joint-zero assets, sample
        // telemetry, or the aerial e-stop effect. They are not scheme selection.
        // CapabilitySchemes and ControlSchemeFactory are intentionally absent.
        static readonly string[] IdAllowlist =
        {
            "Control/RobotProfileFactory.cs",
            "Control/RobotProfile.cs",
            "Control/So101JointZeroConfig.cs",
            "Control/Drive/AerialDriveMapping.cs",
            "Core/RobotDescriptor.cs",
            "Telemetry/TelemetrySamples.cs",
        };

        static string Fixture(string name)
        {
            return Path.Combine(Application.dataPath, "Scripts", "Editor", "Tests", "Fixtures", name);
        }

        static LatchedRobotDescriptor Load(string fileName)
        {
            string json = File.ReadAllText(Fixture(fileName));
            Assert.IsTrue(LatchedRobotDescriptor.TryParse(json, out var descriptor, out var error), error);
            return descriptor;
        }

        static void AssertSafetyUnchanged(RobotProfile profile, LatchedRobotDescriptor descriptor)
        {
            Assert.IsTrue(profile.RequireDeadman);
            Assert.AreEqual(descriptor.Topics.Estop, profile.Topics.EmergencyStop);
            Assert.AreEqual(descriptor.Topics.Heartbeat, profile.Topics.Heartbeat);
            Assert.IsFalse(string.IsNullOrEmpty(profile.Topics.EmergencyStop));
            Assert.IsFalse(string.IsNullOrEmpty(profile.Topics.Heartbeat));
        }

        [Test]
        public void WheeledWithArm_UsesHolonomicDriveAndHandIk()
        {
            var descriptor = Load("capability-wheeled-arm.descriptor.json");
            var profile = RobotProfileFactory.FromDescriptor(descriptor);

            Assert.IsTrue(profile.Capabilities.BaseDrive);
            Assert.IsTrue(profile.Capabilities.Holonomic);
            Assert.IsTrue(profile.Capabilities.Manipulation);
            Assert.IsFalse(profile.Capabilities.Aerial);
            Assert.AreEqual(DriveKind.Mecanum, profile.Drive);
            Assert.IsInstanceOf<MecanumDriveScheme>(ControlSchemeFactory.CreateDriveScheme(profile));
            Assert.IsInstanceOf<HandIK6DofScheme>(ControlSchemeFactory.CreateManipulationScheme(profile, null));
            AssertSafetyUnchanged(profile, descriptor);
            Assert.IsTrue(((DriveSchemeBase)ControlSchemeFactory.CreateDriveScheme(profile)).RequireDeadman);
        }

        [Test]
        public void Legged_UsesQuadrupedStickSchemeAndNoArm()
        {
            var descriptor = Load("capability-legged.descriptor.json");
            var profile = RobotProfileFactory.FromDescriptor(descriptor);

            Assert.IsTrue(profile.Capabilities.LeggedPose);
            Assert.IsTrue(profile.Capabilities.BaseDrive);
            Assert.IsTrue(profile.IsLegged);
            Assert.AreEqual(DriveKind.Quadruped, profile.Drive);
            Assert.IsInstanceOf<MecanumDriveScheme>(ControlSchemeFactory.CreateDriveScheme(profile));
            Assert.IsNull(ControlSchemeFactory.CreateManipulationScheme(profile, null));
            AssertSafetyUnchanged(profile, descriptor);
        }

        [Test]
        public void Aerial_UsesFlightSchemeEvenWhenDriveKindIsFixedBase()
        {
            var descriptor = Load("capability-aerial.descriptor.json");
            Assert.AreEqual("fixed-base", descriptor.Drive.Kind);
            var profile = RobotProfileFactory.FromDescriptor(descriptor);

            Assert.IsTrue(profile.Capabilities.Aerial);
            Assert.IsTrue(profile.IsAerial);
            Assert.IsFalse(profile.IsStationary);
            Assert.Greater(profile.MaxLinVel, 0f);
            Assert.AreEqual(DriveKind.Quadrotor, profile.Drive);
            Assert.IsInstanceOf<AerialDriveScheme>(ControlSchemeFactory.CreateDriveScheme(profile));
            Assert.IsNull(ControlSchemeFactory.CreateManipulationScheme(profile, null));
            AssertSafetyUnchanged(profile, descriptor);
            Assert.IsTrue(((DriveSchemeBase)ControlSchemeFactory.CreateDriveScheme(profile)).RequireDeadman);
        }

        [Test]
        public void Humanoid_UsesBipedStickSchemeAndDualArmFromCapability()
        {
            var descriptor = Load("capability-humanoid.descriptor.json");
            var profile = RobotProfileFactory.FromDescriptor(descriptor);

            Assert.IsTrue(profile.Capabilities.HumanoidPosture);
            Assert.IsTrue(profile.Capabilities.HumanoidLocomotion);
            Assert.IsTrue(profile.Capabilities.DualArm);
            Assert.AreEqual(6, profile.ArmDof);
            Assert.AreEqual(0, profile.LeftArmJointOffset);
            Assert.AreEqual(3, profile.LeftArmJointCount);
            Assert.AreEqual(3, profile.RightArmJointOffset);
            Assert.AreEqual(3, profile.RightArmJointCount);
            Assert.AreEqual("left_shoulder", profile.Joints[profile.LeftArmJointOffset].Name);
            Assert.AreEqual("right_shoulder", profile.Joints[profile.RightArmJointOffset].Name);
            Assert.AreEqual(DriveKind.Bipedal, profile.Drive);
            Assert.IsInstanceOf<MecanumDriveScheme>(ControlSchemeFactory.CreateDriveScheme(profile));
            Assert.IsInstanceOf<DualArmManipulationScheme>(ControlSchemeFactory.CreateManipulationScheme(profile, null));
            Assert.IsFalse(profile.IsAerial);
            AssertSafetyUnchanged(profile, descriptor);
        }

        [Test]
        public void ArmOnly_HasManipulationAndNoDrive()
        {
            var descriptor = Load("capability-arm-only.descriptor.json");
            var profile = RobotProfileFactory.FromDescriptor(descriptor);

            Assert.IsTrue(profile.Capabilities.Manipulation);
            Assert.IsFalse(profile.Capabilities.BaseDrive);
            Assert.IsFalse(profile.Capabilities.Aerial);
            Assert.AreEqual(DriveKind.FixedBase, profile.Drive);
            Assert.AreEqual(0f, profile.MaxLinVel, 0.0001f);
            Assert.IsTrue(string.IsNullOrEmpty(profile.Topics.CmdVel));
            Assert.AreEqual("/arm/joint_commands", profile.Topics.ArmCommands);
            Assert.IsNull(ControlSchemeFactory.CreateDriveScheme(profile));
            Assert.IsInstanceOf<HandIK6DofScheme>(ControlSchemeFactory.CreateManipulationScheme(profile, null));
            Assert.AreEqual(0, profile.LeftArmJointCount);
            Assert.AreEqual(0, profile.RightArmJointCount);
            AssertSafetyUnchanged(profile, descriptor);
        }

        [Test]
        public void Humanoid_PublishesEachArmsOwnJoints()
        {
            var descriptor = Load("capability-humanoid.descriptor.json");
            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            AssertSafetyUnchanged(profile, descriptor);

            var shoulder = profile.Joints[profile.RightArmJointOffset];
            profile.Joints[profile.RightArmJointOffset] = new JointSpec(
                shoulder.Name, shoulder.Label, shoulder.Min, shoulder.Max, 0.25f);
            profile.Topics.ArmCommands = "/arm/joint_commands";

            var scheme = ControlSchemeFactory.CreateManipulationScheme(profile, null);
            Assert.IsInstanceOf<DualArmManipulationScheme>(scheme);
            var published = PublishJointCommands(scheme);

            CollectionAssert.AreEqual(new[]
            {
                "right_shoulder", "right_elbow", "right_grip",
                "left_shoulder", "left_elbow", "left_grip",
            }, published[0].name);
            Assert.AreEqual(6, published[0].position.Length);
            Assert.AreEqual(0.25f, published[0].position[0], 1e-4f);
            for (int i = 1; i < published[0].position.Length; i++)
                Assert.AreEqual(0f, published[0].position[i], 1e-4f, published[0].name[i]);

            Assert.Greater(published[1].position[0], published[0].position[0]);
            Assert.LessOrEqual(published[1].position[0] - published[0].position[0], 0.15f + 1e-4f);
            for (int i = 1; i < published[1].position.Length; i++)
                Assert.AreEqual(published[0].position[i], published[1].position[i], 1e-4f, published[1].name[i]);
        }

        [Test]
        public void DualArm_MapsCapabilityNames_WhenArmsDifferInLength()
        {
            var descriptor = Load("capability-humanoid.descriptor.json");
            descriptor.Arms[0].Name = "starboard";
            descriptor.Arms[0].Joints = new[]
            {
                DescriptorJoint("s_yaw", 0.25),
                DescriptorJoint("s_elbow", -0.5),
            };
            descriptor.Arms[1].Name = "port";
            descriptor.Arms[1].Joints = new[]
            {
                DescriptorJoint("p_yaw", 0.5),
                DescriptorJoint("p_elbow", 0),
                DescriptorJoint("p_grip", -0.25),
            };
            var dual = descriptor.CapabilityParams[CapabilityNames.ManipulationDualArm] as JObject;
            Assert.IsNotNull(dual);
            dual["left"] = "port";
            dual["right"] = "starboard";

            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            Assert.AreEqual(5, profile.ArmDof);
            Assert.AreEqual(0, profile.RightArmJointOffset);
            Assert.AreEqual(2, profile.RightArmJointCount);
            Assert.AreEqual(2, profile.LeftArmJointOffset);
            Assert.AreEqual(3, profile.LeftArmJointCount);
            Assert.AreEqual("s_yaw", profile.Joints[0].Name);
            Assert.AreEqual("p_grip", profile.Joints[4].Name);
            AssertSafetyUnchanged(profile, descriptor);

            profile.Topics.ArmCommands = "/arm/joint_commands";
            var scheme = ControlSchemeFactory.CreateManipulationScheme(profile, null);
            Assert.IsInstanceOf<DualArmManipulationScheme>(scheme);
            var published = PublishJointCommands(scheme);

            CollectionAssert.AreEqual(
                new[] { "s_yaw", "s_elbow", "p_yaw", "p_elbow", "p_grip" },
                published[0].name);
            Assert.AreEqual(0.25f, published[0].position[0], 1e-4f);
            Assert.AreEqual(-0.5f, published[0].position[1], 1e-4f);
            Assert.AreEqual(0.5f, published[0].position[2], 1e-4f);
            Assert.AreEqual(0f, published[0].position[3], 1e-4f);
            Assert.AreEqual(-0.25f, published[0].position[4], 1e-4f);
            Assert.Greater(published[1].position[0], published[0].position[0]);
            Assert.LessOrEqual(published[1].position[0] - published[0].position[0], 0.15f + 1e-4f);
            for (int i = 1; i < published[1].position.Length; i++)
                Assert.AreEqual(published[0].position[i], published[1].position[i], 1e-4f, published[1].name[i]);
        }

        [Test]
        public void PartialRightName_DoesNotGiveTheLeftHandTheRightArm()
        {
            var descriptor = Load("capability-humanoid.descriptor.json");
            SetNeutralArms(descriptor);
            var dual = DualArmBlock(descriptor);
            dual["left"] = "not-the-left";
            dual["right"] = "starboard";

            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            AssertSafetyUnchanged(profile, descriptor);
            Assert.AreEqual(0, profile.RightArmJointOffset);
            Assert.AreEqual(2, profile.RightArmJointCount);
            Assert.AreEqual(2, profile.LeftArmJointOffset);
            Assert.AreEqual(3, profile.LeftArmJointCount);
            Assert.AreEqual("yaw_a", profile.Joints[profile.RightArmJointOffset].Name);
            Assert.AreEqual("yaw_b", profile.Joints[profile.LeftArmJointOffset].Name);

            profile.Topics.ArmCommands = "/arm/joint_commands";
            var scheme = ControlSchemeFactory.CreateManipulationScheme(profile, null);
            Assert.IsInstanceOf<DualArmManipulationScheme>(scheme);
            var published = PublishJointCommands(scheme);
            CollectionAssert.AreEqual(
                new[] { "yaw_a", "elbow_a", "yaw_b", "elbow_b", "grip_b" },
                published[0].name);
            Assert.AreEqual(0.4f, published[0].position[0], 1e-4f);
            Assert.AreEqual(-0.2f, published[0].position[1], 1e-4f);
            Assert.AreEqual(0.1f, published[0].position[2], 1e-4f);
            Assert.AreEqual(0.3f, published[0].position[3], 1e-4f);
            Assert.AreEqual(-0.05f, published[0].position[4], 1e-4f);
            Assert.Greater(published[1].position[0], published[0].position[0]);
            Assert.LessOrEqual(published[1].position[0] - published[0].position[0], 0.15f + 1e-4f);
            for (int i = 1; i < published[1].position.Length; i++)
                Assert.AreEqual(published[0].position[i], published[1].position[i], 1e-4f, published[1].name[i]);
        }

        [Test]
        public void PartialLeftName_KeepsThatArmWhenTheOtherNameIsMissing()
        {
            var descriptor = Load("capability-humanoid.descriptor.json");
            SetNeutralArms(descriptor);
            var dual = DualArmBlock(descriptor);
            dual["left"] = "port";
            dual.Remove("right");

            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            AssertSafetyUnchanged(profile, descriptor);
            Assert.AreEqual(2, profile.LeftArmJointOffset);
            Assert.AreEqual(3, profile.LeftArmJointCount);
            Assert.AreEqual(0, profile.RightArmJointOffset);
            Assert.AreEqual(2, profile.RightArmJointCount);
            Assert.AreEqual("yaw_b", profile.Joints[profile.LeftArmJointOffset].Name);
            Assert.AreEqual("yaw_a", profile.Joints[profile.RightArmJointOffset].Name);

            profile.Topics.ArmCommands = "/arm/joint_commands";
            var scheme = ControlSchemeFactory.CreateManipulationScheme(profile, null);
            Assert.IsInstanceOf<DualArmManipulationScheme>(scheme);
            var published = PublishJointCommands(scheme);
            CollectionAssert.AreEqual(
                new[] { "yaw_a", "elbow_a", "yaw_b", "elbow_b", "grip_b" },
                published[0].name);
        }

        [Test]
        public void AmbiguousDualArm_DoesNotMirrorOrDrive()
        {
            var descriptor = Load("capability-humanoid.descriptor.json");
            SetNeutralArms(descriptor);
            var dual = DualArmBlock(descriptor);
            dual["left"] = "no-such-left";
            dual["right"] = "no-such-right";

            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            AssertSafetyUnchanged(profile, descriptor);
            Assert.AreEqual(0, profile.LeftArmJointCount);
            Assert.AreEqual(0, profile.RightArmJointCount);
            Assert.AreEqual(5, profile.ArmDof);
            Assert.IsNull(ControlSchemeFactory.CreateManipulationScheme(profile, null));

            profile.Topics.ArmCommands = "/arm/joint_commands";
            var scheme = new DualArmManipulationScheme();
            scheme.Configure(profile, null);
            Assert.IsFalse(scheme.IsConfigured);
            var link = new FakeRobotLink();
            var right = new HandPose();
            var left = new HandPose();
            scheme.FeedArmInput(new ArmOperatorInput { EnableRequested = true, DeltaTime = 0.1f });
            Assert.IsFalse(scheme.Tick(ref right, ref left, link));
            Assert.AreEqual(0, link.Published.Count);
        }

        [Test]
        public void BothSidesNamedAsOneArm_DoesNotDrive()
        {
            var descriptor = Load("capability-humanoid.descriptor.json");
            SetNeutralArms(descriptor);
            var dual = DualArmBlock(descriptor);
            dual["left"] = "port";
            dual["right"] = "port";

            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            AssertSafetyUnchanged(profile, descriptor);
            Assert.AreEqual(0, profile.LeftArmJointCount);
            Assert.AreEqual(0, profile.RightArmJointCount);
            Assert.IsNull(ControlSchemeFactory.CreateManipulationScheme(profile, null));
        }

        [Test]
        public void SwappedSideNames_DoNotDriveTheRightArmWithTheLeftHand()
        {
            var descriptor = Load("capability-humanoid.descriptor.json");
            var dual = DualArmBlock(descriptor);
            dual["left"] = "right";
            dual["right"] = "left";

            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            AssertSafetyUnchanged(profile, descriptor);
            Assert.AreEqual(0, profile.LeftArmJointCount);
            Assert.AreEqual(0, profile.RightArmJointCount);
            Assert.IsNull(ControlSchemeFactory.CreateManipulationScheme(profile, null));
        }

        [Test]
        public void ReversedArms_MistypedCapabilityNames_FollowSideTokens()
        {
            var descriptor = Load("capability-humanoid.descriptor.json");
            var rightArm = descriptor.Arms[1];
            var leftArm = descriptor.Arms[0];
            descriptor.Arms[0] = rightArm;
            descriptor.Arms[1] = leftArm;
            var dual = DualArmBlock(descriptor);
            dual["left"] = "missing-left";
            dual["right"] = "missing-right";

            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            AssertSafetyUnchanged(profile, descriptor);
            Assert.AreEqual(0, profile.RightArmJointOffset);
            Assert.AreEqual(3, profile.RightArmJointCount);
            Assert.AreEqual(3, profile.LeftArmJointOffset);
            Assert.AreEqual(3, profile.LeftArmJointCount);
            Assert.AreEqual("right_shoulder", profile.Joints[profile.RightArmJointOffset].Name);
            Assert.AreEqual("left_shoulder", profile.Joints[profile.LeftArmJointOffset].Name);

            profile.Topics.ArmCommands = "/arm/joint_commands";
            var scheme = ControlSchemeFactory.CreateManipulationScheme(profile, null);
            Assert.IsInstanceOf<DualArmManipulationScheme>(scheme);
            var published = PublishJointCommands(scheme);
            CollectionAssert.AreEqual(new[]
            {
                "right_shoulder", "right_elbow", "right_grip",
                "left_shoulder", "left_elbow", "left_grip",
            }, published[0].name);
            Assert.AreEqual(6, published[0].name.Length);
        }

        [Test]
        public void CatalogDualArm_StillMirrorsOneJointList()
        {
            var joints = new JointSpec[7];
            for (int i = 0; i < joints.Length; i++)
                joints[i] = new JointSpec("j" + i, "J" + i, -1.5f, 1.5f, 0f);

            var profile = new RobotProfile
            {
                RobotId = "catalog-humanoid",
                Drive = DriveKind.Bipedal,
                HasArm = true,
                ArmDof = 7,
                Joints = joints,
                Topics = new RobotTopics { ArmCommands = "/arm/joint_commands" },
                HandWorkspaceRadius = 0.4f,
                ArmMaxReach = 0.5f,
            };
            Assert.AreEqual(0, profile.LeftArmJointCount);
            Assert.AreEqual(0, profile.RightArmJointCount);
            Assert.IsTrue(profile.RequireDeadman);

            var scheme = ControlSchemeFactory.CreateManipulationScheme(profile, null);
            Assert.IsInstanceOf<DualArmManipulationScheme>(scheme);
            var published = PublishJointCommands(scheme);

            var expected = new string[14];
            for (int i = 0; i < 7; i++) expected[i] = "r_j" + i;
            for (int i = 0; i < 7; i++) expected[7 + i] = "l_j" + i;
            CollectionAssert.AreEqual(expected, published[0].name);
            Assert.Greater(published[1].position[0], published[0].position[0]);
            Assert.AreEqual(0f, published[1].position[7], 1e-4f);
            Assert.LessOrEqual(published[1].position[0] - published[0].position[0], 0.15f + 1e-4f);
        }

        [Test]
        public void CatalogCategory_DoesNotOverrideDescriptorCapabilities()
        {
            var catalog = new RobotProfile
            {
                RobotId = "catalog-row",
                Category = "drones",
                Drive = DriveKind.Quadrotor,
                IsAerial = true,
                IsDualArm = true,
                IsLegged = true,
            };

            var profile = RobotProfileFactory.FromDescriptor(
                Load("capability-wheeled-arm.descriptor.json"), catalog);

            Assert.AreEqual(DriveKind.Mecanum, profile.Drive);
            Assert.IsFalse(profile.IsAerial);
            Assert.IsFalse(profile.IsDualArm);
            Assert.IsInstanceOf<MecanumDriveScheme>(ControlSchemeFactory.CreateDriveScheme(profile));
            Assert.IsInstanceOf<HandIK6DofScheme>(ControlSchemeFactory.CreateManipulationScheme(profile, null));
        }

        [Test]
        public void UnknownDriveKind_DoesNotGuessDifferential()
        {
            var descriptor = Load("capability-wheeled-arm.descriptor.json");
            descriptor.Drive.Kind = "balloon";
            var profile = RobotProfileFactory.FromDescriptor(descriptor);

            Assert.IsTrue(profile.Capabilities.BaseDrive);
            Assert.AreEqual(0f, profile.MaxLinVel, 0.0001f);
            Assert.IsNull(ControlSchemeFactory.CreateDriveScheme(profile));
            Assert.IsFalse(profile.IsAerial);
        }

        [Test]
        public void LeggedPoseWithoutBaseDrive_DoesNotDrive()
        {
            var descriptor = Load("capability-legged.descriptor.json");
            descriptor.Drive.Kind = "fixed-base";
            descriptor.Drive.Holonomic = false;
            descriptor.Drive.Caps.VxMps = 0;
            descriptor.Drive.Caps.VyMps = 0;
            descriptor.Drive.Caps.WzRadS = 0;
            descriptor.CapabilityParams.Remove(CapabilityNames.BaseDrive);
            descriptor.CapabilityParams.Remove(CapabilityNames.BaseHolonomic);

            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            Assert.IsTrue(profile.Capabilities.LeggedPose);
            Assert.IsFalse(profile.Capabilities.BaseDrive);
            Assert.IsNull(ControlSchemeFactory.CreateDriveScheme(profile));
            Assert.AreEqual(0f, profile.MaxLinVel, 0.0001f);
        }

        [Test]
        public void HumanoidPostureWithoutBaseDrive_DoesNotDrive()
        {
            var descriptor = Load("capability-humanoid.descriptor.json");
            descriptor.Drive.Kind = "fixed-base";
            descriptor.Drive.Holonomic = false;
            descriptor.Drive.Caps.VxMps = 0;
            descriptor.Drive.Caps.VyMps = 0;
            descriptor.Drive.Caps.WzRadS = 0;
            descriptor.CapabilityParams.Remove(CapabilityNames.BaseDrive);
            descriptor.CapabilityParams.Remove(CapabilityNames.BaseHolonomic);

            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            Assert.IsTrue(profile.Capabilities.HumanoidPosture);
            Assert.IsFalse(profile.Capabilities.BaseDrive);
            Assert.IsNull(ControlSchemeFactory.CreateDriveScheme(profile));
            Assert.IsInstanceOf<DualArmManipulationScheme>(
                ControlSchemeFactory.CreateManipulationScheme(profile, null));
        }

        [Test]
        public void UnknownCapability_DoesNotSelectADriveScheme()
        {
            var descriptor = Load("capability-arm-only.descriptor.json");
            descriptor.Arms = new OmniBot.VR.Descriptor.DescriptorArm[0];
            descriptor.CapabilityParams = JObject.Parse("{\"not.a.capability\":{}}");

            var profile = RobotProfileFactory.FromDescriptor(descriptor);
            Assert.IsFalse(profile.Capabilities.BaseDrive);
            Assert.IsFalse(profile.Capabilities.Aerial);
            Assert.IsFalse(profile.Capabilities.Manipulation);
            Assert.IsFalse(profile.Capabilities.LeggedPose);
            Assert.IsNull(ControlSchemeFactory.CreateDriveScheme(profile));
            Assert.IsNull(ControlSchemeFactory.CreateManipulationScheme(profile, null));
            AssertSafetyUnchanged(profile, descriptor);
        }

        [Test]
        public void TryParse_CapabilityParams_AreKept_AndANonObjectIsRejected()
        {
            var obj = JObject.Parse(File.ReadAllText(Fixture("capability-arm-only.descriptor.json")));
            obj["capability_params"] = JObject.Parse("{\"manipulation\":{\"arm_groups\":[\"arm\"],\"ee_frames\":[\"ee\"]}}");
            Assert.IsTrue(LatchedRobotDescriptor.TryParse(obj.ToString(), out var descriptor, out var error), error);
            Assert.IsNotNull(descriptor.CapabilityParams);
            Assert.IsNotNull(descriptor.CapabilityParams["manipulation"]);

            obj["capability_params"] = new JArray();
            Assert.IsFalse(LatchedRobotDescriptor.TryParse(obj.ToString(), out _, out _));

            obj["capability_params"] = new JObject();
            obj["extra"] = 1;
            Assert.IsFalse(LatchedRobotDescriptor.TryParse(obj.ToString(), out _, out _));
        }

        [Test]
        public void BuiltInRobotIds_DoNotAppearInSchemeSelection()
        {
            Assert.IsFalse(Array.IndexOf(IdAllowlist, "Control/CapabilitySchemes.cs") >= 0);
            Assert.IsFalse(Array.IndexOf(IdAllowlist, "Control/ControlSchemeFactory.cs") >= 0);

            string root = Path.Combine(Application.dataPath, "Scripts");
            int scanned = 0;
            foreach (var file in Directory.GetFiles(root, "*.cs", SearchOption.AllDirectories))
            {
                string normalized = file.Replace('\\', '/');
                if (normalized.Contains("/Editor/")) continue;

                bool allowed = false;
                foreach (var suffix in IdAllowlist)
                {
                    if (normalized.EndsWith(suffix, StringComparison.Ordinal))
                    {
                        allowed = true;
                        break;
                    }
                }
                if (allowed) continue;

                scanned++;
                string text = File.ReadAllText(file);
                foreach (var id in BuiltInRobotIds)
                    Assert.IsFalse(text.Contains(id), normalized + " contains " + id);
            }

            Assert.Greater(scanned, 0);

            foreach (var fixture in new[]
            {
                "capability-wheeled-arm.descriptor.json",
                "capability-legged.descriptor.json",
                "capability-aerial.descriptor.json",
                "capability-humanoid.descriptor.json",
                "capability-arm-only.descriptor.json",
            })
            {
                string text = File.ReadAllText(Fixture(fixture));
                foreach (var id in BuiltInRobotIds)
                    Assert.IsFalse(text.Contains(id), fixture + " contains " + id);
            }
        }

        static void SetNeutralArms(LatchedRobotDescriptor descriptor)
        {
            descriptor.Arms[0].Name = "starboard";
            descriptor.Arms[0].Joints = new[]
            {
                DescriptorJoint("yaw_a", 0.4),
                DescriptorJoint("elbow_a", -0.2),
            };
            descriptor.Arms[1].Name = "port";
            descriptor.Arms[1].Joints = new[]
            {
                DescriptorJoint("yaw_b", 0.1),
                DescriptorJoint("elbow_b", 0.3),
                DescriptorJoint("grip_b", -0.05),
            };
        }

        static JObject DualArmBlock(LatchedRobotDescriptor descriptor)
        {
            var dual = descriptor.CapabilityParams[CapabilityNames.ManipulationDualArm] as JObject;
            Assert.IsNotNull(dual);
            return dual;
        }

        static OmniBot.VR.Descriptor.DescriptorJoint DescriptorJoint(string name, double home)
        {
            return new OmniBot.VR.Descriptor.DescriptorJoint
            {
                Name = name,
                Label = name,
                Min = -1.5,
                Max = 1.5,
                Home = home,
            };
        }

        /// <summary>
        /// Real arming: enable, confirm, then two clutched publishes. The first
        /// command is the held pose; the second may jog the right arm.
        /// </summary>
        static JointStateMsg[] PublishJointCommands(IManipulationScheme scheme)
        {
            var link = new FakeRobotLink();
            var right = new HandPose();
            var left = new HandPose();
            var clutch = new ArmOperatorInput
            {
                ClutchHeld = true,
                TrackingGood = false,
                JogAxis = 1f,
                DeltaTime = 0.1f,
            };

            scheme.FeedArmInput(new ArmOperatorInput { EnableRequested = true, DeltaTime = 0.1f });
            scheme.Tick(ref right, ref left, link);
            scheme.FeedArmInput(new ArmOperatorInput { ConfirmRequested = true, DeltaTime = 0.1f });
            scheme.Tick(ref right, ref left, link);
            scheme.FeedArmInput(clutch);
            Assert.IsTrue(scheme.Tick(ref right, ref left, link));
            scheme.FeedArmInput(clutch);
            Assert.IsTrue(scheme.Tick(ref right, ref left, link));

            int count = 0;
            for (int i = 0; i < link.Published.Count; i++)
                if (link.Published[i].Msg is JointStateMsg) count++;

            var msgs = new JointStateMsg[count];
            int index = 0;
            for (int i = 0; i < link.Published.Count; i++)
            {
                if (link.Published[i].Msg is JointStateMsg msg)
                {
                    Assert.AreEqual("/arm/joint_commands", link.Published[i].Topic);
                    msgs[index++] = msg;
                }
            }

            Assert.AreEqual(2, msgs.Length);
            return msgs;
        }
    }
}
