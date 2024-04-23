/// SchoolUtilities.cs
/// Author: Rohith Vishwajith
/// Created 4/22/2024

using UnityEngine;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;

/// <summary>
/// A helper class for SchoolController that handles instantiation. Will handle more later, such as computing an avoidance ray.
/// </summary>
public static class SchoolUtilities
{
    /// <summary>
    /// Instantiate a large number of transforms corresponding to entities with a given position to offset from.
    /// </summary>
    /// <param name="prefab">The transform to instantiate.</param>
    /// <param name="point">The position to offset instantiation from.</param>
    /// <param name="spawnRange">The offset range.</param>
    /// <param name="spawnCount">The number of instantiations.</param>
    /// <returns>An array of instantiated transforms.</returns>
    [BurstCompile]
    public static Transform[] InstantiateEntityTransforms(Transform prefab, float3 point, float2 spawnRange, int spawnCount)
    {
        var transforms = new Transform[spawnCount];
        var random = new Unity.Mathematics.Random(100);
        for (var i = 0; i < transforms.Length; i++)
        {
            var t = GameObject.Instantiate(prefab).transform;
            t.name = "Entity" + i;
            var offsetDist = random.NextFloat(spawnRange.x, spawnRange.y);
            t.position = point + random.NextFloat3Direction() * offsetDist;
            t.LookAt(point, Vector3.up);
            t.forward = t.right;
            transforms[i] = t;
        }
        return transforms;
    }

    static readonly int MAX_INSTANCE_BATCH_SIZE = 1023;

    /// <summary>
    /// If mesh instancing is enabled, render the entities with the given mesh and material
    /// properties using Graphics.RenderMeshInstanced.
    /// </summary>
    [BurstCompile]
    public static void RenderInstances(Transform[] transforms, Mesh mesh, Material material)
    {
        if (transforms == null || mesh == null || material == null)
            return;
        var renderParams = new RenderParams(material);
        var meshesRendered = 0;
        var step = MAX_INSTANCE_BATCH_SIZE;
        for (var i = 0; i < transforms.Length; i += step)
        {
            var instanceCount = math.min(i + step, transforms.Length) - i;
            var matrices = new NativeArray<Matrix4x4>(instanceCount, Allocator.TempJob);
            for (var j = 0; j < instanceCount; j++)
            {
                matrices[j] = transforms[i + j].localToWorldMatrix;
                meshesRendered += 1;
            }
            Graphics.RenderMeshInstanced(renderParams, mesh, 0, matrices);
            matrices.Dispose();
        }
    }

    static readonly RaycastHit[] COLLISION_BUFFER = new RaycastHit[1];

    /// <summary>
    /// Compute a collision avoidance vector provided an entity's position, velocity, and collision
    /// settings.
    /// </summary>
    /// <param name="position">The position of the entity.</param>
    /// <param name="velocity">The velocity of the entity.</param>
    /// <param name="weight">The collision weight (may be removed later).</param>
    /// <param name="castDistance">The raycast distance.</param>
    /// <param name="radius">The radius of the cast. If the radius is 0, a spherecast is used.</param>
    /// <param name="layerMask">The layermask of the cast.</param>
    /// <param name="steerForce">The maximum steer force.</param>
    /// <param name="moveSpeed">The maximum move speed.</param>
    /// <returns>The avoidance vector, adjusted based on the weight, speed, and steerforce.</returns>
    [BurstCompile]
    public static float3 AvoidCollisionForce(float3 position, float3 velocity, float weight, float castDistance, float radius, LayerMask layerMask, float steerForce, float moveSpeed)
    {
        var rotation = quaternion.LookRotation(math.normalize(velocity), SchoolMath.WORLD_UP);
        var dirs = SchoolMath.TURN_DIRS_MED;
        var useSphereCast = radius > 0;
        for (int k = 0; k < dirs.Length; k++)
        {
            var direction = math.rotate(rotation, dirs[k]);
            var ray = new Ray(position, direction);
            if (useSphereCast && Physics.SphereCastNonAlloc(ray, radius, COLLISION_BUFFER, castDistance, layerMask) == 0)
                return weight * SchoolMath.SteerTowards(velocity, direction, steerForce, moveSpeed);
            else if (Physics.RaycastNonAlloc(ray, COLLISION_BUFFER, castDistance, layerMask) == 0)
                return weight * SchoolMath.SteerTowards(velocity, direction, steerForce, moveSpeed);
        }
        // No better direction was found, return the initial direction.
        return velocity;
    }
}