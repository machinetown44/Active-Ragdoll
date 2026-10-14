using System;
using Box3D;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation.Tests
{
    /// <summary>
    /// Ground truth for the tilt-lookahead trajectory (Box3DControllerWrapper.
    /// IntegrateTrajectory): ballistic collide-and-slide slices must (a) sag
    /// into and discover the surface below when airborne, (b) track the ramp
    /// surface while on it, (c) never pass through walls.
    /// World geometry: floor top at y = 0, ramp (30 deg) emerging near x ~ 2.1,
    /// wall at x = 3.
    /// </summary>
    public class MoverLookaheadTests
    {
        private static readonly float3 RampNormal = new float3(-0.5f, 0.8660254f, 0f);

        private static Capsule MoverCap()
        {
            float halfSpan = (1.8f - 2f * 0.4f) * 0.5f;
            return new Capsule
            {
                Center1 = new float3(0f, -halfSpan, 0f),
                Center2 = new float3(0f, halfSpan, 0f),
                Radius = 0.4f,
            };
        }

        /// <summary>Floor top exactly at y = 0.</summary>
        private static World CreateFloorWorld()
        {
            World world = World.Create(WorldDef.Default);
            Body ground = world.CreateBody(BodyDef.Default);
            BoxHull groundHull = BoxHull.Create(40f, 0.5f, 40f);
            ground.CreateHullShape(ShapeDef.Default, in groundHull);
            ground.SetTransform(new float3(0f, -0.5f, 0f), quaternion.identity);
            world.Step(1f / 60f); // warm broadphase
            return world;
        }

        private static void AddRamp(World world)
        {
            Body ramp = world.CreateBody(BodyDef.Default);
            ramp.SetTransform(new float3(3f, 0.5f, 0f), quaternion.RotateZ(math.radians(30f)));
            BoxHull rampHull = BoxHull.Create(3f, 0.25f, 10f);
            ramp.CreateHullShape(ShapeDef.Default, in rampHull);
            world.Step(1f / 60f);
        }

        private static Box3DControllerWrapper NewWrapper()
        {
            var go = new GameObject("traj-test");
            return go.AddComponent<Box3DControllerWrapper>();
        }

        /// <summary>Tilted character walking on flat: the ballistic sag must
        /// contact the floor and report world up as the nearest new support -
        /// this is the evidence that straightens the posture after a ramp.</summary>
        [Test]
        public void SagOntoFlatFloor_FindsWorldUp()
        {
            World world = CreateFloorWorld();
            var wrapper = NewWrapper();
            var cap = MoverCap();
            // 2 cm above the floor, moving forward; baseline = ramp normal
            // (character still carries the ramp tilt).
            var result = wrapper.IntegrateTrajectory(world, cap, QueryFilter.Default,
                new float3(0f, 0.92f, 0f), new float3(2f, 0f, 0f), new float3(0f, -9.81f, 0f),
                RampNormal, false, stackalloc CollisionPlane[8], stackalloc MoverHit[8]);

            world.Destroy();
            UnityEngine.Object.DestroyImmediate(wrapper.gameObject);

            Assert.Greater(result.BestDistance, 0f, "expected some new support along the approach");
            Assert.Greater(result.BestNormal.y, 0.99f,
                $"flat floor must be discovered via the sag, got {result.BestNormal}");
        }

        /// <summary>Character approaching a rising slope: the sag + probe must
        /// find the ramp plane (up-back tilted normal).</summary>
        [Test]
        public void RampAhead_FindsRampSurface()
        {
            World world = CreateFloorWorld();
            AddRamp(world);
            var wrapper = NewWrapper();
            var cap = MoverCap();
            // On the ramp surface at x = 3 (surface y = 0.5), 2 cm float.
            var result = wrapper.IntegrateTrajectory(world, cap, QueryFilter.Default,
                new float3(3f, 1.42f, 0f), new float3(2f, 0f, 0f), new float3(0f, -9.81f, 0f),
                new float3(0f, 1f, 0f), false, stackalloc CollisionPlane[8], stackalloc MoverHit[8]);

            world.Destroy();
            UnityEngine.Object.DestroyImmediate(wrapper.gameObject);

            Assert.Greater(result.BestDistance, 0f, "expected the ramp plane along the approach");
            Assert.Greater(math.dot(math.normalizesafe(result.BestNormal), RampNormal), 0.99f,
                $"ramp surface must be discovered, got {result.BestNormal}");
        }

        /// <summary>Pure into-wall motion: the trajectory stops at the face
        /// (clipped), never passes through.</summary>
        [Test]
        public void WallDeadEnd_NoPassThrough()
        {
            World world = CreateFloorWorld();
            Body wall = world.CreateBody(BodyDef.Default);
            wall.SetTransform(new float3(3f, 2f, 0f), quaternion.identity);
            BoxHull wallHull = BoxHull.Create(0.25f, 2f, 10f);
            wall.CreateHullShape(ShapeDef.Default, in wallHull);
            world.Step(1f / 60f);

            var wrapper = NewWrapper();
            var cap = MoverCap();
            var result = wrapper.IntegrateTrajectory(world, cap, QueryFilter.Default,
                new float3(0f, 0.91f, 0f), new float3(5f, 0f, 0f), new float3(0f, -9.81f, 0f),
                new float3(0f, 1f, 0f), false, stackalloc CollisionPlane[8], stackalloc MoverHit[8]);

            float lastX = wrapper.LookPointCount > 0
                ? wrapper.GetLookPoint(wrapper.LookPointCount - 1).x : 0f;

            world.Destroy();
            UnityEngine.Object.DestroyImmediate(wrapper.gameObject);

            Assert.Greater(wrapper.LookPointCount, 1, "expected the path to advance then stop");
            Assert.Less(lastX, 2.8f,
                $"trajectory must stop at the wall face (near face - radius = 2.35), got x = {lastX}");
        }

        /// <summary>The mover capsule segment is honored as-is: a 60-deg
        /// tilted capsule reaches a waist-high shelf its world-vertical twin
        /// (same origin, same radius) cannot touch. Proves CollideMover has
        /// no hidden upright assumption - query capsules can lean with the
        /// character.</summary>
        [Test]
        public void TiltedMoverCapsule_ContactsWhereVerticalCannot()
        {
            World world = CreateFloorWorld();
            // Shelf: front face at x = 0.75, top at y = 1.0.
            Body shelf = world.CreateBody(BodyDef.Default);
            shelf.SetTransform(new float3(1f, 0.5f, 0f), quaternion.identity);
            BoxHull shelfHull = BoxHull.Create(0.25f, 0.5f, 4f);
            shelf.CreateHullShape(ShapeDef.Default, in shelfHull);
            world.Step(1f / 60f);

            var filter = QueryFilter.Default;
            var origin = new float3(0f, 0.95f, 0f);
            var planes = new CollisionPlane[8];

            // Vertical capsule at this origin: bottom 5 cm above the floor,
            // nearest shelf surface x = 0.4 < 0.75 - no contact anywhere.
            int vertical = world.CollideMover(origin, MoverCap(), filter, planes);

            // Same capsule tilted 60 deg toward +x: the high end (top sphere
            // center at (0.433, 1.15), radius 0.4) reaches x = 0.833 > 0.75
            // inside the shelf's y-band - contact.
            float halfSpan = (1.8f - 2f * 0.4f) * 0.5f;
            float s = math.sin(math.radians(60f)) * halfSpan;
            float c = math.cos(math.radians(60f)) * halfSpan;
            var tilted = new Capsule
            {
                Center1 = new float3(-s, -c, 0f),
                Center2 = new float3(s, c, 0f),
                Radius = 0.4f,
            };
            var hits = new MoverHit[8];
            int tiltedN = world.CollideMover(origin, tilted, filter, planes, hits);

            world.Destroy();

            Assert.Zero(vertical, "vertical capsule must clear the shelf at this origin");
            Assert.Greater(tiltedN, 0, "tilted capsule must reach the shelf");
            Assert.Greater(hits[0].Point.y, 0.7f,
                $"contact must be at the shelf top region via the tilted high end, got y = {hits[0].Point.y}");
        }
    }
}
