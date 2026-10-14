using System;
using Unity.Mathematics;
using Box3D;

namespace Box3DArticulation
{
    /// <summary>Foot support/contact model — port of the snowboard project's BodyContactModel
    /// (itself a verbatim copy of the proven BoardContactModel.Evaluate pipeline: graded plastic
    /// foundation, footprint clipping, centroid crossfade, impulse friction cone with gravity
    /// bias, torque damping). The only functional change for feet: the plow term is removed
    /// (plow = carving; feet don't carve). The grip falloff by bone attitude (foot articulation
    /// limits) is NOT here — it lives in FootStabilizer, which feeds an attitude-modulated mu
    /// into frictionBase/frictionEdge, so this core stays identical to the proven model.
    ///
    /// Frame contract (caller-supplied, orientation-free — any attitude): forward/right/up are
    /// the body's own axes. For a box bone (long axis Y): forward = body Y, right = body X,
    /// up = body Z; halfLength = long half-extent, halfWidth = transverse half-extent. The
    /// footprint rectangle (forward·halfLength x right·halfWidth at the origin) is the body's
    /// material silhouette; the origin IS the body COM. Isotropic friction: pass
    /// frictionBase == frictionEdge (the cone's ellipse becomes a circle). The friction basis
    /// is tangent-projected (kept from the body model) so the cone stays exact at any attitude.
    ///
    /// "Lift distance": snowDepth is the virtual surface height above the terrain facets —
    /// support engages within the band, not at a hard plane. On rigid ground the graded
    /// capacity ramp sigma(z) = sigmaC·min(1, z/zGrad) IS the soft-ish spring, and the plateau
    /// is the load cap: the foot sinks until ramp+capacity balance entry dynamics.</summary>
    public static class FootContactModel
    {
        public struct Result
        {
            public float3 Impulse;            // force * dt, applied at the origin (COM)
            public float3 Torque;             // about the origin (COM)
            public int PlaneCount;
            public float MaxPenetration;
            public float BuriedFraction;      // plateau-equivalent buried fraction
            public float3 TotalNormalForce;
            public float3 Centroid;           // effective anchor (world)
            public float3 FrictionForce;
            public int PolyCount;             // clipped buried-footprint polygon (polyOut)
        }

        /// <summary>Evaluates the contact forces for one plane set. dt = fixed step; the impulse
        /// is force x dt since Box3D bodies take impulses, not forces.</summary>
        public static Result Evaluate(
            ReadOnlySpan<CollisionPlane> planes,
            ReadOnlySpan<MoverHit> hits,
            int count,
            float3 origin,
            float3 velocity,
            float3 angularVelocity,
            float3 bodyUp,
            float3 bodyForward,
            float3 bodyRight,
            float snowDepth,
            float bearingForce,     // plateau capacity at full burial
            float zGrad,            // capacity ramp length: sigma(z) = sigmaC·min(1, z/zGrad)
            float dampingCoeff,
            float frictionBase,
            float frictionEdge,
            float halfLength,
            float halfWidth,
            float maxWingForce,
            float dt,
            float torqueDampMul,    // rotational damping knob
            float centroidFade,     // anchor crossfade rate
            float massKg,           // loaded mass: friction cone works in impulse space
            float3 gravity,         // world gravity accel: friction needs the upcoming step's bias
            ref float3 smoothNormal,
            float normalSlewDegPerSec,
            Span<float3> polyOut)   // debug: buried polygon verts (caller-owned scratch)
        {
            var result = new Result();
            if (count <= 0) return result;

            // Pass 1 (direction + reference): aggregate facet normals, pick the deepest contact
            // as the depth-field anchor. NO gate here — the origin's own clearance says nothing
            // about burial (tip-down bone: origin high above the band, tip deeply buried).
            // Gating moved to the clipped footprint (area > 0). Facet normals are reoriented to
            // the gravity-up side; blend weights are the contacts' true geometric burial (plane
            // offset) — a buried facet ALWAYS has offset > 0, so the blend never starves.
            float deepestPen = -1e9f;
            float3 deepestPoint = origin;
            float3 rawNormal = 0f;
            for (int i = 0; i < count; i++)
            {
                float3 nF = hits[i].Normal;
                if (nF.y < 0f) nF = -nF;   // gravity-layer terrain: facet faces up
                float vClear = math.dot(nF, origin - hits[i].Point) / math.max(nF.y, 0.2f);
                float pen = snowDepth - vClear;
                if (pen > deepestPen) { deepestPen = pen; deepestPoint = hits[i].Point; }
                rawNormal += nF * math.max(planes[i].Plane.Offset, 0f);
                if (pen > 0f)
                {
                    result.MaxPenetration = math.max(result.MaxPenetration, pen);
                    result.PlaneCount++;
                }
            }

            // Pass 2: smooth the aggregated direction, compose force along it. One-sided:
            // magnitudes are preserved but the direction is shared, so no plane can pull.
            // Slew-rate limit insures against discrete facet switches.
            float3 target = math.normalizesafe(rawNormal, smoothNormal);
            float3 nS = normalSlewDegPerSec > 0f
                ? RotateToward(smoothNormal, target, math.radians(normalSlewDegPerSec) * dt)
                : target;
            smoothNormal = nS;
            // Depth field (positive = buried): d(x) = snowDepth - dot(nGeo, x - deepestPoint).
            // GEOMETRY uses the RAW aggregated normal, never the slewed one: slew lag is a
            // force-direction concern only — measuring depth against a stale normal tilts the
            // field on flat ground and kicks roll every step.
            float3 nGeo = math.normalizesafe(rawNormal, nS);
            float3 fwd = bodyForward * halfLength;
            float3 right = bodyRight * halfWidth;
            Span<float3> poly = stackalloc float3[8];
            Span<float3> tmp = stackalloc float3[8];
            poly[0] = origin + fwd + right;
            poly[1] = origin + fwd - right;
            poly[2] = origin - fwd - right;
            poly[3] = origin - fwd + right;
            int n = 4;
            int m = 0;
            for (int i = 0; i < n; i++)
            {
                float3 a = poly[i];
                float3 b = poly[(i + 1) % n];
                float da = snowDepth - math.dot(nGeo, a - deepestPoint);
                float db = snowDepth - math.dot(nGeo, b - deepestPoint);
                if (da >= 0f) tmp[m++] = a;
                if ((da > 0f) != (db > 0f))
                    tmp[m++] = a + (b - a) * (da / (da - db));
            }
            n = m;
            for (int i = 0; i < n; i++) poly[i] = tmp[i];
            if (n < 3) return result;

            // Shoelace fan over the clipped polygon in the orthonormal (right, forward) body
            // basis. Depth-graded capacity: pressure ramps with local depth up to the plateau,
            // so the deep side of a tilted footprint presses harder — that per-triangle
            // gradient is the restoring attitude moment, present at ANY attitude. Integrate
            // sigma*dA and sigma*dA*x in the same pass — one loop, no allocations.
            float area2 = 0f, cx = 0f, cz = 0f, fAcc = 0f, fx = 0f, fz = 0f;
            float invZGrad = zGrad > 1e-4f ? 1f / zGrad : 1e4f;
            float dOrigin = snowDepth - math.dot(nGeo, origin - deepestPoint);
            for (int i = 0; i < n; i++)
            {
                float3 a = poly[i];
                float3 b = poly[(i + 1) % n];
                float ax = math.dot(a - origin, bodyRight);
                float az = math.dot(a - origin, bodyForward);
                float bx = math.dot(b - origin, bodyRight);
                float bz = math.dot(b - origin, bodyForward);
                float da = snowDepth - math.dot(nGeo, a - deepestPoint);
                float db = snowDepth - math.dot(nGeo, b - deepestPoint);
                float cross = ax * bz - bx * az;
                area2 += cross;
                cx += (ax + bx) * cross;
                cz += (az + bz) * cross;
                float zTri = (da + db + dOrigin) / 3f; // fan apex = body origin
                float s = math.saturate(zTri * invZGrad); // sigma/sigmaC per triangle
                fAcc += cross * s;
                fx += (ax + bx) * cross * s;
                fz += (az + bz) * cross * s;
            }
            float area = 0.5f * math.abs(area2);
            float fullArea = 4f * halfLength * halfWidth;
            if (area <= 1e-6f) return result; // footprint gate: buried area, not origin clearance
            float sigmaC = bearingForce / math.max(fullArea, 1e-6f); // plateau pressure
            float fRaw = 0.5f * sigmaC * math.abs(fAcc);             // graded support magnitude
            float fracEff = fRaw / math.max(bearingForce, 1e-6f);    // plateau-equivalent fraction
            float invFAcc = math.abs(fAcc) > 1e-9f ? 1f / (3f * fAcc) : 0f;
            float3 cB = origin + bodyRight * (fx * invFAcc) + bodyForward * (fz * invFAcc);

            // Centroid crossfade (anti-pump): when the bone is planar-aligned with the surface,
            // the buried-wedge centroid sits far off-center — tiny attitude flips move the
            // torque arm sides every substep, and a one-step-delayed damping force at the
            // flipped anchor pumps energy. Fade the anchor toward the body center as the buried
            // fraction shrinks. Crossfades the damping anchor and the support torque arm; the
            // support MAGNITUDE stays unchanged.
            float fade = math.saturate(fracEff * math.max(centroidFade, 1e-3f));
            float3 cEff = origin + (cB - origin) * fade;

            // Support: graded plateau magnitude, rate damping on the effective anchor's
            // velocity (rotation-aware), one-sided (no suction when rising).
            float3 vC = velocity + math.cross(angularVelocity, cEff - origin);
            float vN = math.dot(vC, nS);
            float fSupp = math.clamp(fRaw - dampingCoeff * fracEff * vN, 0f, maxWingForce);
            float3 supportF = nS * fSupp;

            // Load-scaled rotational damping of ALL axes — anisotropic per body geometry:
            // pitch (about right, arm ~ halfLength) gets halfLength^2, roll gets halfWidth^2,
            // yaw gets the squared diagonal.
            float wPitch = math.dot(angularVelocity, bodyRight);
            float wRoll = math.dot(angularVelocity, bodyForward);
            float wYaw = math.dot(angularVelocity, bodyUp);
            float dampK = dampingCoeff * fracEff * math.max(torqueDampMul, 0f);
            float3 torqueDamp = -bodyRight * (wPitch * halfLength * halfLength * dampK)
								-bodyForward * (wRoll * halfWidth * halfWidth * dampK)
								-bodyUp * (wYaw * (halfLength * halfLength + halfWidth * halfWidth) * dampK);

            // Torque from the SAME force (one accounting): effective anchor x support.
            float3 torque = math.cross(cEff - origin, supportF) + torqueDamp;

            // Surface-frame friction against the smoothed surface plane — impulse-clamp
            // friction cone, elliptical in body frame (isotropic when base == edge: circle).
            // Applied at COM — torque-free. Cone semantics: the impulse that would zero
            // tangential slip THIS step is scaled into the mu-ellipse — inside it the body
            // comes to TRUE rest (static friction holds; kills slope creep), outside it
            // degrades to ordinary sliding. Uniform SCALE of the stop vector, not a
            // per-component clamp. Budget is fRAW — the undamped geometric support demand:
            // the viscous support term taxes the same coins friction needs, and a resting
            // foot's tiny vN must not shrink its stiction. fRaw vanishes with burial: no
            // bite while flying.
            float3 vt = velocity - nS * math.dot(velocity, nS);
            // Gravity bias: the world integrates gravity AFTER our pre-step impulse, so a
            // stop impulse that only cancels incoming velocity still leaves v = g_t*dt at
            // the end of every step (slope creep). Cancel what gravity WILL add this step.
            float3 gT = gravity - nS * math.dot(gravity, nS);
            float3 e1 = math.normalizesafe(bodyForward - nS * math.dot(bodyForward, nS), bodyRight);
            float3 e2 = math.cross(nS, e1);
            float vF = math.dot(vt, e1) + dt * math.dot(gT, e1);
            float vR = math.dot(vt, e2) + dt * math.dot(gT, e2);
            float invDt = 1f / math.max(dt, 1e-6f);
            float stopF = -massKg * vF * invDt / math.max(frictionBase * fRaw, 1e-6f);
            float stopR = -massKg * vR * invDt / math.max(frictionEdge * fRaw, 1e-6f);
            float ellipse = math.sqrt(stopF * stopF + stopR * stopR);
            float k = ellipse > 1f ? 1f / ellipse : 1f;
            float3 fFric = -e1 * (stopF * k * frictionBase * fRaw)
                           -e2 * (stopR * k * frictionEdge * fRaw);

            result.TotalNormalForce = supportF;
            result.BuriedFraction = fracEff;
            result.Centroid = cEff;
            result.FrictionForce = fFric;
            result.PolyCount = math.min(n, polyOut.Length);
            for (int i = 0; i < result.PolyCount; i++) polyOut[i] = poly[i];
            result.Torque = torque;
            result.Impulse = (supportF + fFric) * dt;
            return result;
        }

        /// <summary>Rotates 'from' toward 'to' by at most maxRad radians about their mutual axis.
        /// Both inputs are unit vectors.</summary>
        private static float3 RotateToward(float3 from, float3 to, float maxRad)
        {
            float cos = math.clamp(math.dot(from, to), -1f, 1f);
            float angle = math.acos(cos);
            if (angle <= maxRad || angle < 1e-6f) return to;
            float3 axis = math.normalizesafe(math.cross(from, to), new float3(0f, 1f, 0f));
            quaternion q = quaternion.AxisAngle(axis, maxRad);
            return math.rotate(q, from);
        }
    }
}
