using System;
using Newtonsoft.Json.Linq;
using LatchedRobotDescriptor = OmniBot.VR.Descriptor.RobotDescriptor;

namespace OmniBot.VR.Control
{
    /// <summary>
    /// Capability names that choose a VR control scheme. The strings match
    /// <c>ohho.capabilities</c>. A name outside this set does not select a scheme.
    /// </summary>
    public static class CapabilityNames
    {
        public const string BaseDrive = "base.drive";
        public const string BaseHolonomic = "base.holonomic_drive";
        public const string LeggedPose = "legged.pose";
        public const string HumanoidLocomotion = "humanoid.locomotion";
        public const string HumanoidPosture = "humanoid.posture";
        public const string Aerial = "aerial";
        public const string Manipulation = "manipulation";
        public const string ManipulationDualArm = "manipulation.dual_arm";
        public const string Gripper = "end_effector.gripper";
        public const string DexHand = "end_effector.dex_hand";
    }

    /// <summary>
    /// Capabilities implied by one latched descriptor. Built from the drive block,
    /// the arm list, and <c>capability_params</c> names. Catalog category is not an input.
    /// </summary>
    public sealed class DescriptorCapabilitySet
    {
        public bool FromDescriptor;
        public bool BaseDrive;
        public bool Holonomic;
        public bool LeggedPose;
        public bool HumanoidPosture;
        public bool HumanoidLocomotion;
        public bool Aerial;
        public bool Manipulation;
        public bool DualArm;
        public bool Gripper;
        public bool DexHand;

        public static DescriptorCapabilitySet From(LatchedRobotDescriptor descriptor)
        {
            var set = new DescriptorCapabilitySet { FromDescriptor = true };
            if (descriptor == null) return set;

            string kind = descriptor.Drive != null ? descriptor.Drive.Kind : null;
            if (!string.Equals(kind, "fixed-base", StringComparison.Ordinal))
            {
                set.BaseDrive = true;
                if (descriptor.Drive != null && descriptor.Drive.Holonomic)
                    set.Holonomic = true;
            }

            if (descriptor.Arms != null && descriptor.Arms.Length > 0)
                set.Manipulation = true;

            ApplyParams(set, descriptor.CapabilityParams);
            return set;
        }

        static void ApplyParams(DescriptorCapabilitySet set, JObject parms)
        {
            if (parms == null) return;
            foreach (var prop in parms.Properties())
            {
                switch (prop.Name)
                {
                    case CapabilityNames.BaseDrive:
                        set.BaseDrive = true;
                        break;
                    case CapabilityNames.BaseHolonomic:
                        set.BaseDrive = true;
                        set.Holonomic = true;
                        break;
                    case CapabilityNames.LeggedPose:
                        set.LeggedPose = true;
                        break;
                    case CapabilityNames.HumanoidPosture:
                        set.HumanoidPosture = true;
                        break;
                    case CapabilityNames.HumanoidLocomotion:
                        set.HumanoidLocomotion = true;
                        break;
                    case CapabilityNames.Aerial:
                        set.Aerial = true;
                        break;
                    case CapabilityNames.Manipulation:
                        set.Manipulation = true;
                        break;
                    case CapabilityNames.ManipulationDualArm:
                        set.Manipulation = true;
                        set.DualArm = true;
                        break;
                    case CapabilityNames.Gripper:
                        set.Gripper = true;
                        break;
                    case CapabilityNames.DexHand:
                        set.DexHand = true;
                        break;
                }
            }
        }
    }

    /// <summary>
    /// Maps descriptor capabilities onto the existing drive and manipulation schemes.
    /// Unknown or absent drive capabilities select nothing.
    /// </summary>
    public static class CapabilitySchemes
    {
        /// <summary>
        /// Pick a <see cref="DriveKind"/> the drive-scheme switch already knows.
        /// Returns false when the capability set does not justify a scheme.
        /// </summary>
        public static bool TrySelectDrive(DescriptorCapabilitySet caps, DriveKind parsed, out DriveKind selected)
        {
            selected = DriveKind.FixedBase;
            if (caps == null || !caps.FromDescriptor) return false;

            // Flight is the aerial capability. The descriptor drive enum has no
            // aerial kind, so a fixed-base block plus this capability is a copter.
            if (caps.Aerial)
            {
                selected = DriveKind.Quadrotor;
                return true;
            }

            // Posture and legged poses are not a ground scheme by themselves.
            // Sticks require base.drive and a kind this client can map.
            if (!caps.BaseDrive) return false;

            switch (parsed)
            {
                case DriveKind.Mecanum:
                    selected = DriveKind.Mecanum;
                    return true;
                case DriveKind.Ackermann:
                    selected = DriveKind.Ackermann;
                    return true;
                case DriveKind.Differential:
                case DriveKind.SkidSteer:
                case DriveKind.Tracked:
                case DriveKind.RockerBogie:
                    selected = caps.Holonomic ? DriveKind.Mecanum : parsed;
                    return true;
                case DriveKind.Quadruped:
                case DriveKind.Hexapod:
                case DriveKind.Bipedal:
                case DriveKind.Thruster:
                    selected = parsed;
                    return true;
                default:
                    return false;
            }
        }

        public static bool AllowsManipulation(DescriptorCapabilitySet caps)
        {
            return caps != null && (caps.Manipulation || caps.DualArm || caps.Gripper || caps.DexHand);
        }
    }
}
