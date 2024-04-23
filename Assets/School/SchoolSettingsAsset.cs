/// SchoolSettingsAsset.cs
/// Author: Rohith Vishwajith
/// Created 4/21/2024

using UnityEngine;

/// <summary>
/// A scriptable object used to store flocking weights, collision rules, and rendering settings
/// used for flocking simulations in SchoolController.
/// </summary>
[CreateAssetMenu]
public class SchoolSettings : ScriptableObject
{
    [Header("Movement")]
    public float minSpeed = 4f;
    public float maxSpeed = 9.5f;
    public float maxSteerForce = 10f;

    [Header("Rule Weights")]
    public float perceptionRadius = 4f;
    public float avoidanceRadius = 2f;
    public float alignWeight = 1.5f;
    public float cohesionWeight = 1;
    public float separateWeight = 2.5f;

    [Header("Seeking")]
    public float targetWeight = 1f;

    [Header("Collisions")]
    public bool enableCollisions = true;
    public float avoidCollisionWeight = 20f;
    public bool skipCollisionFrames = false;
    public int collisionFrameSkips = 1;
    public LayerMask collisionMask = 1;
    public float collisionCheckDistance = 4f;
    public float collisionCheckRadius = 0.25f;

    [Header("Performance")]
    public bool enableParallelJobs = true;
    public int parallelJobBatchCount = 4;

    [Header("Rendering")]
    public bool useMeshInstancing = false;
}