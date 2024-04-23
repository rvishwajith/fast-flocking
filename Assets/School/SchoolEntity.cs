/// SchoolBoid.cs
/// Author: Rohith Vishwajith
/// Created 4/21/2024

using UnityEngine;
using Unity.Mathematics;

/// <summary>
/// SchoolBoid:
/// A simple struct containing the data of a single entity in a school, which is controlled 
/// by SchoolController. Designed to be thread-safe for Jobs-based workflow.
/// </summary>
struct SchoolBoid
{
    public int id;
    public float3 position;
    public float3 velocity;

    public Transform target;
    public bool checkCollision;

    // FIXME: Remove the 6 fields below later.
    public float3 acceleration;
    public Transform transform;
    public int detectedNeighbors;
    public float3 neighborCenter;
    public float3 neighborHeading;
    public float3 avoidHeading;

    public quaternion rotation { get { return quaternion.LookRotation(velocity, SchoolMath.WORLD_UP); } }
    public Matrix4x4 worldMatrix { get { return transform.localToWorldMatrix; } }
    public float3 forward { get { return math.normalize(velocity); } }
}