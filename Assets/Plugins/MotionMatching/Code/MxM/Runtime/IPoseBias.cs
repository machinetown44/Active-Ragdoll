using UnityEngine;

namespace MxM
{
    /// <summary>
    /// Optional hook to modify the runtime query pose before the pose search
    /// jobs consume it. Implemented outside the plugin (e.g. physics feedback);
    /// a null PoseBias field on MxMAnimator = stock behavior.
    /// </summary>
    public interface IPoseBias
    {
        void BiasPose(ref PoseData pose);
    }
}
