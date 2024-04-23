/// SchoolUtilities.cs
/// Author: Rohith Vishwajith
/// Created 4/22/2024

using UnityEngine;
using Unity.Mathematics;
using Unity.Burst;

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
    // [BurstCompile(CompileSynchronously = true)]
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
}