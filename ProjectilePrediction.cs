using System.Collections.Generic;
using UnityEngine;

public static class ProjectilePrediction {
    private static List<float> rootBuffer = new List<float>();

    public readonly struct Target {
        private readonly Vector3 position;
        private readonly Vector3 velocity;
        public readonly Vector3 acceleration;
        private Target(Vector3 position, Vector3 velocity, Vector3 acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
        /// <summary>
        /// Generates a read-only shot target that represents an actor that either shoots or gets shot by projectiles.
        /// </summary>
        /// <param name="position">The position the projectile hits or comes out of.</param>
        /// <param name="velocity">The velocity of the actor, would be Vector3.zero for stationary actors.</param>
        /// <param name="grounded">Indicates if the actor is falling through the air with regular gravity or not. Use GetShotTargetCustom if the actor has arbitrary gravity.</param>
        /// <returns>A Target to be used with ShotConfiguration</returns>
        public static Target GetShotTarget(Vector3 position, Vector3 velocity, bool grounded = true) {
            return new Target(position, velocity, grounded ? Vector3.zero : Physics.gravity);
        }
        
        /// <summary>
        /// Generates a read-only shot target that represents an actor that either shoots or gets shot by projectiles.
        /// </summary>
        /// <param name="position">The position the projectile hits or comes out of.</param>
        /// <param name="velocity">The velocity of the actor, would be Vector3.zero for stationary actors.</param>
        /// <param name="acceleration">The acceleration of the actor, it would be Vector3.zero if it's on the ground. Physics.gravity if they're falling at Earth's gravity.</param>
        /// <returns>A Target to be used with ShotConfiguration</returns>
        public static Target GetShotTargetCustom(Vector3 position, Vector3 velocity, Vector3 acceleration) {
            return new Target(position, velocity, acceleration);
        }

        /// <summary>
        /// Moves the target forward or backward in time before sampling its position and velocity.
        /// </summary>
        /// <param name="time">How much time to simulate before sampling the position. 0 will sample its current position.</param>
        /// <param name="trajPosition">The position of the projectile at the given time.</param>
        /// <param name="trajVelocity">The velocity of the projectile at the given time.</param>
        /// <returns></returns>
        public void SampleTrajectory(float time, out Vector3 trajPosition, out Vector3 trajVelocity) {
            trajPosition = position + velocity * time + acceleration * (0.5f * time * time);
            trajVelocity = velocity + acceleration * time;
        }
    }
    public readonly struct ShotQuery {
        public readonly Target shooter;
        public readonly Target target;
        
        public readonly float projectileMuzzleSpeed;
        public readonly Vector3 projectileInheritedVelocity;
        public readonly Vector3 projectileAcceleration;

        private ShotQuery(Target shooter, Target target, float projectileMuzzleSpeed, Vector3 projectileAcceleration, Vector3 projectileInheritedVelocity) {
            this.shooter = shooter;
            this.target = target;
            this.projectileMuzzleSpeed = projectileMuzzleSpeed;
            this.projectileAcceleration = projectileAcceleration;
            this.projectileInheritedVelocity = projectileInheritedVelocity;
        }

        /// <summary>
        /// Creates a shot configuration that will be used for future calculations. Use TryGetAimDirectionForFastball and TryGetAimDirectionForMortar on it after.
        /// </summary>
        /// <param name="shooter">The barrel of the gun, where the projectile comes out.</param>
        /// <param name="target">The point on the target we're trying to hit, the bullseye.</param>
        /// <param name="projectileMuzzleSpeed">How fast the projectile is going when it leaves the barrel. Pure projectiles only, no rockets or air-friction allowed!</param>
        /// <param name="projectileAcceleration">The gravity applied to the projectile, will be Physics.gravity unless the projectile has custom physics.</param>
        /// <param name="projectileInheritedVelocity">The velocity of the projectile inherited from the Shooter, this is separate from the shooter's velocity. This will be Vector3.zero for the vast majority of cases. 50% of the shooter's speed in Tribes. 100% of the shooters speed in reality.</param>
        /// <returns>A queryable object that can give predictions on a shot.</returns>
        public static ShotQuery GetShotQuery(Target shooter, Target target, float projectileMuzzleSpeed, Vector3 projectileAcceleration, Vector3 projectileInheritedVelocity = default(Vector3)) {
            return new ShotQuery(shooter, target, projectileMuzzleSpeed, projectileAcceleration, projectileInheritedVelocity);
        }

        /// <summary>
        /// Finds hit timings via intersections between an expanding sphere representing all possible shots and a parabolic trajectory representing the relative motion of the target.
        /// Can give 0, 1, or 2 solutions. The results are sorted.
        /// </summary>
        /// <param name="shotTimings">A buffer to hold all the timings that you must allocate beforehand.</param>
        /// <param name="delay">How long we wait before the shooter makes the shot.</param>
        /// <param name="tolerance">How accurate the timings need to be to give up searching for a solution.</param>
        /// <param name="maxIterations">How many iterations before giving up trying to hit the tolerance.</param>
        /// <returns>The number of valid solutions found.</returns>
        public int TryGetPossibleHitTimings(List<float> shotTimings, float delay = 0f, float tolerance = 0.025f, int maxIterations = 12) {
            target.SampleTrajectory(delay, out var targetPosition, out var targetVelocity);
            shooter.SampleTrajectory(delay, out var shooterPosition, out var shooterVelocity);
            Vector3 p = targetPosition - shooterPosition;
            Vector3 v = targetVelocity - projectileInheritedVelocity;
            Vector3 a = target.acceleration - projectileAcceleration;
            float t4 = (a.x * a.x + a.y * a.y + a.z * a.z) / 4f;
            float t3 = a.x * v.x + a.y * v.y + a.z * v.z;
            float t2 = v.x * v.x + p.x * a.x + v.y * v.y + p.y * a.y + v.z * v.z + p.z * a.z - projectileMuzzleSpeed * projectileMuzzleSpeed;
            float t1 = 2f * (p.x * v.x + p.y * v.y + p.z * v.z);
            float t0 = p.x * p.x + p.y * p.y + p.z * p.z;
            shotTimings.Clear();
            Polynomials.RootSolver.GetRootsForQuartic(shotTimings, t4, t3, t2, t1, t0, tolerance, maxIterations);
            shotTimings.RemoveAll(IsInvalidRoot);
            shotTimings.Sort();
            return shotTimings.Count;
        }
        // Roots that are negative are backward in time, we discard those.
        private static bool IsInvalidRoot(float x) => x < 0f;
        
        /// <summary>
        /// Gets the lowest, fastest arc to hit the target.
        /// </summary>
        /// <param name="aimDirection">The normalized aim direction needed to fire.</param>
        /// <param name="delay">How long before the projectile fires.</param>
        /// <returns>True if the shot is possible, false otherwise.</returns>
        public bool TryGetAimDirectionForFastball(out Vector3 aimDirection, float delay = 0f, float tolerance = 0.025f, int maxIterations = 12) {
            int possibleHits = TryGetPossibleHitTimings(rootBuffer, delay, tolerance, maxIterations);
            if (possibleHits == 0) {
                aimDirection = Vector3.up;
                return false;
            }
            aimDirection = ConvertHitTimingToAimDirection(rootBuffer[0], delay);
            return true;
        }
        
        /// <summary>
        /// Gets the slowest, tallest arc to hit the target.
        /// </summary>
        /// <param name="aimDirection">The normalized aim direction needed to fire.</param>
        /// <param name="delay">How long before the projectile fires.</param>
        /// <returns>True if the shot is possible, false otherwise.</returns>
        public bool TryGetAimDirectionForMortar(out Vector3 aimDirection, float delay = 0f, float tolerance = 0.025f, int maxIterations = 12) {
            int possibleHits = TryGetPossibleHitTimings(rootBuffer, delay, tolerance, maxIterations);
            if (possibleHits == 0) {
                aimDirection = Vector3.up;
                return false;
            }
            aimDirection = ConvertHitTimingToAimDirection(rootBuffer[^1], delay);
            return true;
        }
        
        /// <summary>
        /// Converts a hit timing provided by TryGetPossibleHitTimings to a usable aim direction using standard projectile equations. Only useful when used in conjunction with `TryGetPossibleHitTimings`
        /// </summary>
        /// <param name="t">The time until the projectile hits the target.</param>
        /// <param name="delay">The time before the projectile is fired.</param>
        /// <returns>The direction to shoot to hit the target.</returns>
        public Vector3 ConvertHitTimingToAimDirection(float t, float delay) {
            target.SampleTrajectory(delay, out var targetPosition, out var targetVelocity);
            shooter.SampleTrajectory(delay, out var shooterPosition, out var shooterVelocity);
            Vector3 p = targetPosition - shooterPosition;
            Vector3 v = targetVelocity - projectileInheritedVelocity;
            Vector3 a = target.acceleration - projectileAcceleration;
            Vector3 aimTarget = p + v * t + a * (0.5f * t * t);
            return aimTarget.normalized;
        }
    }
}
