/// SchoolMath.cs
/// Author: Rohith Vishwajith
/// Created 4/21/2024

using System;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Burst;

/// <summary>
/// A helper class for school simulation calculations, such as:
/// - Computing acceleration / velocity values.
/// - Computing and caching avoidance rays.
/// - Getting constants using Unity.Mathematics (SIMD) types.
/// All functions in this class are static and burst compiled.
/// </summary>
public static class SchoolMath
{
    // GPU RENDERING / INSTANCING -----------------------------------------------------------------

    /// <summary>
    /// The maximum instance count for drawing instanced meshes using DrawMeshIndirect.
    /// </summary>
    public static readonly int MAX_INSTANCE_BATCH_SIZE = 1023;

    // PHYSICS & MATH -----------------------------------------------------------------------------

    /// <summary>
    /// The layer mask for school entities.
    /// </summary>
    public static readonly LayerMask SCHOOL_LAYER = LayerMask.NameToLayer("School");

    /// <summary>
    /// The equivalent to Vector3.up but as a float3.
    /// </summary>
    public static readonly float3 WORLD_UP = new(0, 1, 0);

    /// <summary>
    /// REMOVEME. Precomputed world-space turn directions for obstacle avoidance, using an array
    /// of Vector3 instead of float3.
    /// </summary>
    public static readonly Vector3[] TURN_DIRS_V3 = ComputeTurnRays(100);

    /// <summary>
    /// Precomputed world-space turn directions for obstacle avoidance, with LOW precision.
    /// Note: LOW precision = 50 samples.
    /// </summary>
    public static readonly float3[] TURN_DIRS_LOW = ComputeTurnRaysF3(50);

    /// <summary>
    /// World-space turn directions for obstacle avoidance, with MEDIUM precision (100 samples).
    /// </summary>
    public static readonly float3[] TURN_DIRS_MED = ComputeTurnRaysF3(100);

    /// <summary>
    /// World-space turn directions for obstacle avoidance, with HIGH precision (300 samples).
    /// </summary>
    public static readonly float3[] TURN_DIRS_HIGH = ComputeTurnRaysF3(300);

    [BurstCompile]
    public static Vector3[] ComputeTurnRays(int samples)
    {
        var goldenRatio = (1 + Mathf.Sqrt(5)) / 2;
        var angleIncr = Mathf.PI * 2 * goldenRatio;
        var dirs = new Vector3[samples];
        for (int i = 0; i < samples; i++)
        {
            var t = (float)i / samples;
            var incl = Mathf.Acos(1 - 2 * t);
            var azimuth = angleIncr * i;
            dirs[i] = new(Mathf.Sin(incl) * Mathf.Cos(azimuth),
                Mathf.Sin(incl) * Mathf.Sin(azimuth),
                Mathf.Cos(incl));
        }
        return dirs;
    }

    /// <summary>
    /// Compute avoidance rays (a sphere of rays arranged with a 3-D fibonacci spiral) from a
    /// given number of samples.
    /// </summary>
    /// <param name="samples"></param>
    /// <returns></returns>
    [BurstCompile]
    public static float3[] ComputeTurnRaysF3(int samples = 100)
    {
        var goldenRatio = (1 + Mathf.Sqrt(5)) / 2;
        var angleIncr = Mathf.PI * 2 * goldenRatio;
        var dirs = new float3[samples];
        for (int i = 0; i < samples; i++)
        {
            var t = (float)i / samples;
            var incl = Mathf.Acos(1 - 2 * t);
            var azimuth = angleIncr * i;
            dirs[i] = new(Mathf.Sin(incl) * Mathf.Cos(azimuth),
                Mathf.Sin(incl) * Mathf.Sin(azimuth),
                Mathf.Cos(incl));
        }
        return dirs;
    }

    [BurstCompile]
    public static float3 SteerTowards(float3 direction, float3 targetDirection, float steerForce, float speed)
    {
        // if (math.length(vector) == 0 && math.length(velocity) == 0)
        //     return new(0, 0, 1);
        if (math.length(targetDirection) == 0)
            return direction;
        // else if (math.length(velocity) == 0)
        //     return vector;
        var dir = (speed * math.normalize(targetDirection)) - direction;
        if (math.length(dir) == 0)
            return targetDirection;
        // FIXME: Change this to a clamp of 0.0001 and remove the length check above.
        return math.normalize(dir) * math.clamp(math.length(dir), 0, steerForce);
    }

    /// <summary>
    /// Creates a NativeArray of a given size and populate it with only the given value/
    /// </summary>
    /// <param name="value">The value to fill the NativeArray.</param>
    /// <param name="size">The size of the NativeArray.</param>
    /// <param name="allocator">The NativeArray allocator (TempJob by default).</param>
    /// <returns></returns>
    [BurstCompile]
    public static NativeArray<float> ToNativeArray(float value, int size, Allocator allocator = Allocator.TempJob)
    {
        var arr = new float[size];
        // for (int i = 0; i < size; ++i)
        //     arr[i] = value;
        arr.Populate<float>(0, size, value);
        return new NativeArray<float>(arr, allocator);
    }

    /// <summary>
    /// A helper method to quickly populate a large array with the same value.
    /// Source: https://stackoverflow.com/questions/1014005/how-to-populate-instantiate-a-c-sharp-array-with-a-single-value
    /// </summary>
    /// <typeparam name="T">The type of the array.</typeparam>
    /// <param name="array">The input array.</param>
    /// <param name="start">The starting index.</param>
    /// <param name="count">The number of values to populate.</param>
    /// <param name="value">The value to populate the array with.</param>
    [BurstCompile]
    public static void Populate<T>(this T[] array, int start, int count, T value)
    {
        const int gap = 16;
        var i = start;
        if (count <= gap * 2)
        {
            while (count > 0)
            {
                array[i] = value;
                count--;
                i++;
            }
            return;
        }
        var avail = gap;
        count -= gap;
        do
        {
            array[i] = value;
            i++;
            --avail;
        } while (avail > 0);

        avail = gap;
        while (true)
        {
            Array.Copy(array, start, array, i, avail);
            i += avail;
            count -= avail;
            avail *= 2;
            if (count <= avail)
            {
                Array.Copy(array, start, array, i, count);
                break;
            }
        }
    }
}