using UnityEngine;
using OmniBot.VR.Core;

namespace OmniBot.VR.Control.Manip
{
    /// <summary>
    /// Dual-arm manipulation — for humanoid robots (Unitree G1, Figure 02) where
    /// two hands drive two independent arms. The right hand retargets to the
    /// right arm, the left hand to the left arm. Each arm is solved independently
    /// via <see cref="FabrikSolver"/> (generic chains, since humanoid arms rarely
    /// match the SO-101 analytic geometry). Pinch on either hand drives that
    /// hand's gripper.
    ///
    /// Publishes a single <see cref="JointStateMsg"/> with both arms' joint names
    /// and positions to <c>profile.Topics.ArmCommands</c> at 20 Hz. Layout is
    /// right-arm joints, then left-arm joints. A dual-arm descriptor publishes
    /// each arm's own names. A descriptor that does not resolve two different
    /// arms publishes no joint command. A catalog profile whose <see cref="RobotProfile.Joints"/>
    /// array is one arm is mirrored with <c>r_</c> and <c>l_</c> prefixes. The
    /// arm-enable toggle (left thumbs-up) enables/disables both arms together.
    /// </summary>
    public sealed class DualArmManipulationScheme : IManipulationScheme
    {
        private const float PublishHz = 20f;

        private RobotProfile _profile;
        private Transform _workspaceOrigin;
        private readonly ArmSessionGate _gate = new ArmSessionGate();
        private float _publishInterval;
        private float _publishTimer;
        private float _maxDelta = RobotDescriptor.MaxJointDeltaRad;
        private ArmOperatorInput _input;
        private ArmOperatorInput _frame;
        private bool _suppressMotionThisPublish;
        private int _jogJoint;
        private string _status = "Arm off";

        // Two independent IK solvers (one per arm)
        private FabrikSolver _rightSolver;
        private FabrikSolver _leftSolver;
        private float[] _rightAngles;
        private float[] _leftAngles;

        // Joint name layout: [right arm joints..., left arm joints...]
        private string[] _allJointNames;
        private int _rightJointOffset;
        private int _rightJointCount;
        private int _leftJointOffset;
        private int _leftJointCount;

        // Workspace origins for each arm (offset left/right from the center origin)
        private Vector3 _rightArmOffset;
        private Vector3 _leftArmOffset;

        public bool IsConfigured => _profile != null && _rightSolver != null;
        public bool ArmEnabled => _gate.Flow.TorqueEnabled;
        public ArmEnablePhase EnablePhase => _gate.Flow.Phase;
        public bool ClutchEngaged => _gate.Flow.ClutchEngaged;
        public string StatusText => _status;
        public string HudHint => "Right trigger: clutch · Two hands: dual arm · Left X / thumbs-up: enable · REAL: Left X again";

        public void Configure(RobotProfile profile, Transform workspaceOrigin)
        {
            _profile = profile;
            _workspaceOrigin = workspaceOrigin;
            _gate.Bind(profile);
            _publishInterval = 1f / PublishHz;
            _publishTimer = 0f;
            _suppressMotionThisPublish = false;
            _jogJoint = 0;
            _maxDelta = RobotDescriptor.ClampJointDelta(profile != null ? profile.MaxJointDeltaRad : 0f);

            // A resolved descriptor drives each span under its own names.
            // A catalog profile has one joint list and is mirrored with r_/l_ prefixes.
            // A descriptor dual-arm with no resolved spans publishes nothing.
            if (HasDistinctArms(profile))
            {
                BindArms(
                    profile.RightArmJointOffset,
                    profile.RightArmJointCount,
                    profile.LeftArmJointOffset,
                    profile.LeftArmJointCount,
                    prefixSides: false);
            }
            else if (profile.Capabilities != null
                     && profile.Capabilities.FromDescriptor
                     && profile.Capabilities.DualArm)
            {
                _rightSolver = null;
                _leftSolver = null;
                _allJointNames = null;
                return;
            }
            else
            {
                int count = profile.Joints != null ? profile.Joints.Length : 7;
                BindArms(0, count, 0, count, prefixSides: true);
            }

            // Shoulder offsets: arms are spaced ~0.36 m apart on a humanoid torso.
            _rightArmOffset = new Vector3(+0.18f, 0f, 0f);
            _leftArmOffset = new Vector3(-0.18f, 0f, 0f);
            RefreshStatus();
        }

        public void FeedArmInput(ArmOperatorInput input) => _input = input;

        public bool Tick(ref HandPose rightHand, ref HandPose leftHand, IRobotLink link)
        {
            if (!IsConfigured || _profile == null || link == null || !link.IsConnected) return false;

            _frame = _input;
            ClearEdges();
            var signal = _gate.Consume(_frame);
            ArmEnablePublish.Apply(signal, _profile, link);
            RefreshStatus();

            if (_gate.ClutchRising) _suppressMotionThisPublish = true;
            if (!_gate.Flow.MotionAllowed)
            {
                _suppressMotionThisPublish = false;
                return false;
            }

            float dt = _frame.DeltaTime > 0f ? _frame.DeltaTime : Time.deltaTime;
            _publishTimer += dt;
            if (_publishTimer < _publishInterval) return false;
            _publishTimer -= _publishInterval;

            if (_suppressMotionThisPublish)
            {
                _suppressMotionThisPublish = false;
            }
            else if (!_frame.TrackingGood)
            {
                ApplyJog(dt);
            }
            else if (_workspaceOrigin != null)
            {
                if (rightHand.Tracked)
                    _rightAngles = StepArm(_rightSolver, rightHand, _rightArmOffset, _rightAngles, _rightJointOffset, _rightJointCount);
                if (leftHand.Tracked)
                    _leftAngles = StepArm(_leftSolver, leftHand, _leftArmOffset, _leftAngles, _leftJointOffset, _leftJointCount);
            }

            // Combine both arms into one JointState message
            int rightCount = _rightAngles.Length;
            int leftCount = _leftAngles.Length;
            var allAngles = new float[rightCount + leftCount];
            System.Array.Copy(_rightAngles, 0, allAngles, 0, rightCount);
            System.Array.Copy(_leftAngles, 0, allAngles, rightCount, leftCount);

            if (_profile.Topics == null || string.IsNullOrEmpty(_profile.Topics.ArmCommands)) return false;
            var msg = new JointStateMsg((string[])_allJointNames.Clone(), allAngles);
            link.Publish(_profile.Topics.ArmCommands, msg);
            return true;
        }

        public void Stop(IRobotLink link)
        {
            var signal = _gate.Flow.Disarm();
            ArmEnablePublish.Apply(signal, _profile, link);
            _publishTimer = 0f;
            _suppressMotionThisPublish = false;
            RefreshStatus();
        }

        public void SetArmEnabled(bool enabled, IRobotLink link)
        {
            var signal = enabled ? _gate.Flow.RequestEnable() : _gate.Flow.Disarm();
            ArmEnablePublish.Apply(signal, _profile, link);
            RefreshStatus();
        }

        private float StepCap()
        {
            float cap = _maxDelta;
            float hard = SafetyCaps.JointStepLimit(_profile);
            if (hard > 0f && hard < cap) return hard;
            return cap;
        }

        private void ApplyJog(float dt)
        {
            if (_rightAngles == null || _rightAngles.Length == 0) return;
            if (_frame.CycleJointPressed)
                _jogJoint = (_jogJoint + 1) % _rightAngles.Length;
            var target = (float[])_rightAngles.Clone();
            int index = Mathf.Clamp(_jogJoint, 0, target.Length - 1);
            target[index] = _rightAngles[index] + JointJog.DeltaRad(_frame.JogAxis, dt);
            JointRateLimiter.StepTowards(_rightAngles, target, StepCap(), Limits(_rightJointOffset, _rightJointCount, true), Limits(_rightJointOffset, _rightJointCount, false));
        }

        private float[] StepArm(FabrikSolver solver, HandPose hand, Vector3 armOffset, float[] current, int jointOffset, int jointCount)
        {
            var held = current != null ? (float[])current.Clone() : new float[jointCount];
            float[] solved = SolveArm(solver, hand, _workspaceOrigin, armOffset, held, jointOffset, jointCount, _profile);
            JointRateLimiter.StepTowards(held, solved, StepCap(), Limits(jointOffset, jointCount, true), Limits(jointOffset, jointCount, false));
            return held;
        }

        private float[] Limits(int offset, int count, bool min)
        {
            var values = new float[count];
            var joints = _profile != null ? _profile.Joints : null;
            for (int i = 0; i < count && joints != null && offset + i < joints.Length; i++)
                values[i] = min ? joints[offset + i].Min : joints[offset + i].Max;
            return values;
        }

        private void RefreshStatus()
        {
            string joint = "joint";
            if (_rightJointCount > 0 && _profile != null && _profile.Joints != null)
            {
                int index = Mathf.Clamp(_jogJoint, 0, _rightJointCount - 1);
                int flat = _rightJointOffset + index;
                if (flat >= 0 && flat < _profile.Joints.Length)
                    joint = _profile.Joints[flat].Name;
            }
            _status = _gate.Flow.Describe(_frame.TrackingGood, joint);
        }

        private void ClearEdges()
        {
            _input.EnableRequested = false;
            _input.ConfirmRequested = false;
            _input.DisarmRequested = false;
            _input.CancelRequested = false;
            _input.CycleJointPressed = false;
        }

        // ── Helpers ───────────────────────────────────────────────────────────────

        private float[] SolveArm(
            FabrikSolver solver,
            HandPose hand,
            Transform origin,
            Vector3 armOffset,
            float[] currentAngles,
            int jointOffset,
            int jointCount,
            RobotProfile profile)
        {
            // Hand position relative to the arm's shoulder
            Vector3 armBase = origin.position + armOffset;
            float scale = profile.ArmMaxReach / profile.HandWorkspaceRadius;

            Vector3 targetRel = (hand.Position - armBase) * scale;
            targetRel = Vector3.ClampMagnitude(targetRel, solver.TotalReach * 0.98f);

            // Solve in a frame relative to the arm base
            Vector3 localTarget = origin.InverseTransformPoint(armBase + targetRel);
            // Shift so the base is at origin for the solver
            Vector3 armBaseLocal = origin.InverseTransformPoint(armBase);
            localTarget -= armBaseLocal;

            float[] newAngles = solver.Solve(localTarget, currentAngles);
            float step = SafetyCaps.JointStepLimit(profile);

            // Gripper from pinch (last joint of this arm), then keep every joint inside the step cap.
            bool gripped = false;
            if (newAngles.Length > 0 && TryJoint(profile, jointOffset, jointCount, jointCount - 1, out var grip))
            {
                gripped = true;
                int last = newAngles.Length - 1;
                float prev = currentAngles != null && last < currentAngles.Length ? currentAngles[last] : newAngles[last];
                newAngles[last] = SafetyCaps.ClampJoint(
                    prev,
                    Mathf.Lerp(grip.Max, grip.Min, Mathf.Clamp01(hand.PinchStrength)),
                    grip.Min,
                    grip.Max,
                    step);
            }

            for (int i = 0; i < newAngles.Length; i++)
            {
                if (gripped && i == newAngles.Length - 1)
                    continue;
                float prev = currentAngles != null && i < currentAngles.Length ? currentAngles[i] : newAngles[i];
                newAngles[i] = SafetyCaps.ClampJoint(prev, newAngles[i], float.NegativeInfinity, float.PositiveInfinity, step);
            }

            return newAngles;
        }

        internal static bool HasDistinctArms(RobotProfile profile)
        {
            if (profile?.Joints == null) return false;
            int leftOffset = profile.LeftArmJointOffset;
            int leftCount = profile.LeftArmJointCount;
            int rightOffset = profile.RightArmJointOffset;
            int rightCount = profile.RightArmJointCount;
            if (!SpanFits(profile.Joints.Length, leftOffset, leftCount)) return false;
            if (!SpanFits(profile.Joints.Length, rightOffset, rightCount)) return false;
            int leftEnd = leftOffset + leftCount;
            int rightEnd = rightOffset + rightCount;
            return leftEnd <= rightOffset || rightEnd <= leftOffset;
        }

        private static bool SpanFits(int length, int offset, int count)
        {
            return offset >= 0 && count > 0 && offset <= length && count <= length - offset;
        }

        private void BindArms(int rightOffset, int rightCount, int leftOffset, int leftCount, bool prefixSides)
        {
            _rightJointOffset = rightOffset;
            _rightJointCount = rightCount;
            _leftJointOffset = leftOffset;
            _leftJointCount = leftCount;

            var rightJoints = CopySpan(rightOffset, rightCount);
            var leftJoints = CopySpan(leftOffset, leftCount);
            _rightSolver = new FabrikSolver();
            _leftSolver = new FabrikSolver();
            _rightSolver.Configure(rightJoints, _profile.ArmLinkLengths);
            _leftSolver.Configure(leftJoints, _profile.ArmLinkLengths);

            _rightAngles = Homes(rightJoints);
            _leftAngles = Homes(leftJoints);

            var rightNames = JointNames(rightOffset, rightCount, prefixSides ? "r_" : null);
            var leftNames = JointNames(leftOffset, leftCount, prefixSides ? "l_" : null);
            _allJointNames = new string[rightNames.Length + leftNames.Length];
            System.Array.Copy(rightNames, 0, _allJointNames, 0, rightNames.Length);
            System.Array.Copy(leftNames, 0, _allJointNames, rightNames.Length, leftNames.Length);
        }

        private JointSpec[] CopySpan(int offset, int count)
        {
            var joints = new JointSpec[count];
            for (int i = 0; i < count; i++)
                joints[i] = _profile.Joints[offset + i];
            return joints;
        }

        private string[] JointNames(int offset, int count, string prefix)
        {
            var names = new string[count];
            for (int i = 0; i < count; i++)
            {
                string name = _profile.Joints[offset + i].Name;
                names[i] = string.IsNullOrEmpty(prefix) ? name : prefix + name;
            }
            return names;
        }

        private static float[] Homes(JointSpec[] joints)
        {
            var angles = new float[joints.Length];
            for (int i = 0; i < joints.Length; i++)
                angles[i] = joints[i].Home;
            return angles;
        }

        private static bool TryJoint(RobotProfile profile, int offset, int count, int localIndex, out JointSpec joint)
        {
            joint = default;
            if (profile?.Joints == null || localIndex < 0 || localIndex >= count) return false;
            int index = offset + localIndex;
            if (index < 0 || index >= profile.Joints.Length) return false;
            joint = profile.Joints[index];
            return true;
        }
    }
}
