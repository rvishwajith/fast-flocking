// SchoolComputeAcceleration.cs
// Author: Rohith Vishwajith
// Created 4/21/2024

using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;

/// <summary>
/// A modified version of the acceleration computation for SchoolController using multithreading 
/// via Unity's jobs system (TODO).
/// </summary>
[BurstCompile]
public struct SchoolComputeAccelerationJob : IJobParallelFor
{
    // Required inputs (readonly):
    // - movement: position, velocity
    // - distances: detectRadius, avoidRadius
    // - weights: alignWeight, cohesionWeight, separateWeight
    // - speeds: speed
    [ReadOnly] public NativeArray<float3> positions;
    [ReadOnly] public NativeArray<float3> velocities;

    [ReadOnly] public NativeArray<float> detectRadii;
    [ReadOnly] public NativeArray<float> avoidRadii;

    [ReadOnly] public NativeArray<float> alignWeights;
    [ReadOnly] public NativeArray<float> cohesionWeights;
    [ReadOnly] public NativeArray<float> separateWeights;

    [ReadOnly] public NativeArray<float> steerForces;
    [ReadOnly] public NativeArray<float> maxSpeeds;

    // Job outputs:
    // - accelerations (from neighbors only)
    [WriteOnly] public NativeArray<float3> accelerations;

    [BurstCompile]
    public void Execute(int i)
    {
        // Store result in Jobs results array.
        // accelerations[i] = ComputeAccelerationUsingPhysics(i);
        accelerations[i] = ComputeAccelerationIterative(i);
    }

    // float3 ComputeAccelerationUsingPhysics(int i)
    // {
    //     var pos = positions[i];
    //     var velocity = velocities[i];

    //     var detectRadius = detectRadii[i];
    //     var avoidRadius = avoidRadii[i];

    //     var alignWeight = alignWeights[i];
    //     var cohesionWeight = cohesionWeights[i];
    //     var separateWeight = separateWeights[i];

    //     var steerForce = steerForces[i];
    //     var maxSpeed = maxSpeeds[i];

    //     var neighbors = Physics.OverlapSphere(pos, detectRadius, SchoolMath.SCHOOL_LAYER);
    //     var detectedNeighbors = neighbors.Length - 1;
    //     var neighborHeading = new float3();
    //     var neighborCenter = new float3();
    //     var avoidHeading = new float3();
    //     var acceleration = new float3();
    //     for (var j = 0; j < detectedNeighbors; j++)
    //     {
    //         float3 neighborPos = neighbors[j].transform.position;
    //         var dist = math.distance(pos, neighborPos);
    //         neighborCenter += neighborPos;
    //         neighborHeading += neighbors[i].transform.forward;
    //         if (dist > 0.01f && dist <= avoidRadius)
    //         {

    //         }
    //     }
    // }

    /// <summary>
    /// Compute the neighbors' accleration using a simple distance check.
    /// </summary>
    /// <returns></returns>
    [BurstCompile]
    float3 ComputeAccelerationIterative(int i)
    {
        // Cache input data (just to make code more readable).
        var pos = positions[i];
        var velocity = velocities[i];

        var detectRadius = detectRadii[i];
        var avoidRadius = avoidRadii[i];

        var alignWeight = alignWeights[i];
        var cohesionWeight = cohesionWeights[i];
        var separateWeight = separateWeights[i];

        var steerForce = steerForces[i];
        var maxSpeed = maxSpeeds[i];

        // Store data needed for computing acceleration.
        int detectedNeighbors = 0;
        var neighborHeading = new float3();
        var neighborCenter = new float3();
        var avoidHeading = new float3();
        var acceleration = new float3();

        for (int j = 0; j < positions.Length; j++)
        {
            if (i == j)
                continue;
            var neighborPos = positions[j];
            var dist = math.distance(pos, neighborPos);
            if (dist > 0 && dist <= detectRadius)
            {
                detectedNeighbors += 1;
                neighborHeading += math.normalize(velocities[j]);
                if (dist <= avoidRadius)
                    avoidHeading += (pos - neighborPos) / (dist * dist);
            }
        }

        // Neighbors were detected, compute standards boids forces.
        if (detectedNeighbors != 0)
        {
            neighborCenter /= detectedNeighbors;
            var align = alignWeight * SchoolMath.SteerTowards(
                velocity, neighborHeading, steerForce, maxSpeed);
            var toCenter = cohesionWeight * SchoolMath.SteerTowards(
                velocity, neighborCenter - pos, steerForce, maxSpeed);
            var separate = separateWeight * SchoolMath.SteerTowards(
                velocity, avoidHeading, steerForce, maxSpeed);
            acceleration += align + toCenter + separate;
        }
        return acceleration;
    }
}