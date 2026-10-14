using UnityEngine;

namespace Box3DArticulation
{
    /// Central switch for spawn/rig diagnostics verbosity. Manual sessions
    /// (heuristic batch, WaveParamTester) opt in; training runs stay quiet.
    /// Rare hard anomalies (spawn exceptions) log regardless of this flag.
    public static class WaveDiagnostics
    {
        public static bool Verbose;
    }
}