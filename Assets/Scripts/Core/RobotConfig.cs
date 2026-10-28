namespace OmniBot.VR.Core
{
    /// <summary>
    /// Physical limits and arm geometry. Topic names are not constants:
    /// connected robots take them from the latched RobotDescriptor, and offline
    /// browsing takes them from the catalog profile.
    /// </summary>
    public static class RobotConfig
    {
        // Topic names live on the catalog profile and the latched descriptor.
        // OmniBot's catalog arm command is /arm/joint_commands/teleop (main's
        // mux input). RobotConfig no longer declares TOPIC_* constants.

        // ── Physical limits ──────────────────────────────────────────────────
        /// <summary>Maximum base linear velocity in m/s.</summary>
        public const float MAX_LINEAR_VEL  = 0.2f;

        /// <summary>Maximum base angular velocity in rad/s.</summary>
        public const float MAX_ANGULAR_VEL = 1.0f;

        /// <summary>Rate at which /cmd_vel/teleop is published (Hz).</summary>
        public const float CMD_VEL_PUBLISH_HZ = 20f;

        /// <summary>Rate at which /arm/joint_commands/teleop is published (Hz).</summary>
        public const float ARM_PUBLISH_HZ = 20f;

        // ── Arm geometry ─────────────────────────────────────────────────────
        // Single source is RobotDescriptor.So101. These fields are copies so
        // existing callers keep compiling. Do not introduce a second set of
        // literals here. Kinematic zeros are 0 rad; OHH-94 owns servo ticks.
        public static readonly string[] ARM_JOINT_NAMES = (string[])RobotDescriptor.So101.JointNames.Clone();
        public static readonly float[] ARM_JOINT_MIN = (float[])RobotDescriptor.So101.JointMin.Clone();
        public static readonly float[] ARM_JOINT_MAX = (float[])RobotDescriptor.So101.JointMax.Clone();

        /// <summary>Upper-arm link length (shoulder to elbow), metres.</summary>
        public static readonly float ARM_LINK1_LENGTH = RobotDescriptor.So101.LinkLengths[0];

        /// <summary>Forearm link length (elbow to wrist), metres.</summary>
        public static readonly float ARM_LINK2_LENGTH = RobotDescriptor.So101.LinkLengths[1];

        /// <summary>Wrist/end-effector link length, metres.</summary>
        public static readonly float ARM_LINK3_LENGTH = RobotDescriptor.So101.LinkLengths[2];

        /// <summary>Height of arm shoulder above robot base, metres.</summary>
        public static readonly float ARM_BASE_HEIGHT = RobotDescriptor.So101.BaseHeight;

        /// <summary>
        /// Radius of the VR hand-tracking workspace sphere, in metres.
        /// Hand position within this sphere is mapped linearly to the arm workspace.
        /// </summary>
        public static readonly float HAND_WORKSPACE_RADIUS = RobotDescriptor.So101.HandWorkspaceRadius;
    }
}
