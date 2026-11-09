using System;
using System.IO;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using OmniBot.VR.Control;
using OmniBot.VR.Control.Drive;
using OmniBot.VR.Control.Manip;
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
            AssertSafetyUnchanged(profile, descriptor);
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
    }
}
