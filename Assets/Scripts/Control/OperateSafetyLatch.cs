namespace OmniBot.VR.Control
{
    /// <summary>
    /// Operate-screen e-stop latch. Grip e-stop stays latched until left X even
    /// when this is clear; releasing the grips does not clear it. While this is
    /// set, left X is refused and autonomous mode changes are not published.
    /// Only an explicit operate reset clears this latch. This never turns a
    /// stop off by itself.
    /// </summary>
    public static class OperateSafetyLatch
    {
        public static bool UiEStopLatched { get; private set; }

        public static void Latch() => UiEStopLatched = true;

        public static void Clear() => UiEStopLatched = false;
    }

    /// <summary>
    /// Pure safety decisions shared by the drive scheme and the operate bar.
    /// </summary>
    public static class OperateSafetyRules
    {
        /// <summary>
        /// Grip release does not clear a stop. The operate hold
        /// (<paramref name="uiLatched"/>) and the grip latch both wait for an
        /// explicit reset.
        /// </summary>
        public static bool GripReleaseClearsStop(bool uiLatched)
        {
            // Grip release is not a reset. One grip is the deadman and the right
            // grip is turbo. The operate latch has its own explicit reset.
            if (uiLatched)
                return false;
            return false;
        }

        /// <summary>Nav2 / policy / teleop publishes wait until both stops are clear.</summary>
        public static bool ModePublishAllowed(bool uiLatched, bool gripStopActive)
            => !uiLatched && !gripStopActive;

        /// <summary>Reset cannot clear a stop while both grips are still held.</summary>
        public static bool ResetAllowed(bool gripHeld) => !gripHeld;
    }
}
