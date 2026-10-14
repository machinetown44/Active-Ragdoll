namespace Box3D.Hybrid
{
    /// <summary>A replay component that can be scrubbed — implemented by <see cref="Box3DReplayer"/>
    /// (wireframe) and <see cref="Box3DVisualReplayer"/> (real objects) so one editor timeline drives both.</summary>
    public interface IReplayTimeline
    {
        bool IsCreated { get; }
        bool IsPlaying { get; }
        int Frame { get; }
        int FrameCount { get; }
        bool HasDiverged { get; }
        int DivergeFrame { get; }

        void SeekFrame(int frame);
        void StepFrame();
        bool TogglePlay();
        void Restart();
    }
}
