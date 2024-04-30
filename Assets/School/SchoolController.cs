/// SchoolController.cs
/// Author: Rohith Vishwajith
/// Created 4/21/2024

using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

/// <summary>
/// SchoolController:
/// Fish school simulator based on the Boids algorithm by Craig Reynolds, with additional features
/// such as target seeking and obstacle avoidance.
/// Will use the Unity Jobs system to maximize performance while keeping all data on the CPU.
/// </summary>
class SchoolController : MonoBehaviour
{
    [SerializeField][Range(1, 10000)] int spawnCount = 500;
    [SerializeField] SchoolSettings settings = null;
    [SerializeField] Transform prefab = null;
    [SerializeField] float2 spawnRange = new(5, 10);

    // Remove this later.
    int frameCount = 0;

    // Use GPU instancing if enabled in settings.
    Mesh instanceMesh;
    Material instanceMaterial;
    Transform[] transforms;

    /// <summary>
    /// Array of entities representing a single fish. Stores values for position, velocity, and
    /// will later also store weight data, etc.
    /// </summary>
    SchoolBoid[] entities;

    // Native containers for jobs.
    NativeArray<float3> positions;
    NativeArray<float3> velocities;
    NativeArray<float> detectRadii;
    NativeArray<float> detectAngles;
    NativeArray<float> avoidRadii;
    NativeArray<float> alignWeights;
    NativeArray<float> cohesionWeights;
    NativeArray<float> separateWeights;
    NativeArray<float> steerForces;
    NativeArray<float> maxSpeeds;
    NativeArray<float3> accelerations;

    /// <summary>
    /// If mesh instancing is enabled and the instances have a mesh filter component, copy the
    /// mesh data from it and disable/remove and rendering data from the GameObject.
    /// </summary>
    void SetupMeshInstancing()
    {
        if (settings == null || !settings.useMeshInstancing || entities.Length == 0)
            return;
        // Cache the material and mesh.
        var entityTransform = entities[0].transform;
        if (entityTransform.TryGetComponent<MeshFilter>(out var meshFilter))
            instanceMesh = meshFilter.mesh;
        if (entityTransform.TryGetComponent<MeshRenderer>(out var meshRenderer))
            instanceMaterial = meshRenderer.material;
        // Disable/remove the mesh renderers for each instance.
        for (var i = 0; i < entities.Length; i++)
        {
            entities[i].transform.GetComponent<MeshRenderer>().enabled = false;
            // Destroy(entities[i].transform.GetComponent<MeshRenderer>());
        }
    }

    void AllocateJobContainers()
    {
        var size = entities.Length;
        var allocator = Allocator.Persistent;
        positions = new(size, allocator);
        velocities = new(size, allocator);
        for (int i = 0; i < size; i++)
        {
            positions[i] = entities[i].position;
            velocities[i] = entities[i].velocity;
        }
        detectAngles = SchoolMath.ToNativeArray(settings.perceptionAngle, size, allocator);
        detectRadii = SchoolMath.ToNativeArray(settings.perceptionRadius, size, allocator);
        avoidRadii = SchoolMath.ToNativeArray(settings.avoidanceRadius, size, allocator);
        alignWeights = SchoolMath.ToNativeArray(settings.alignWeight, size, allocator);
        cohesionWeights = SchoolMath.ToNativeArray(settings.cohesionWeight, size, allocator);
        separateWeights = SchoolMath.ToNativeArray(settings.separateWeight, size, allocator);
        steerForces = SchoolMath.ToNativeArray(settings.maxSteerForce, size, allocator);
        maxSpeeds = SchoolMath.ToNativeArray(settings.maxSpeed, size, allocator);
        accelerations = new(size, allocator);
    }

    /// <summary>
    /// When the school controller is rmeoved (the scene ends), dispose of all NativeArrays since
    /// they must be manually de-allocated.
    /// </summary>
    void OnDestroy()
    {
        // Debug.Log("Killing school controller, disposing of allocated jobs data.");
        positions.Dispose();
        velocities.Dispose();
        detectRadii.Dispose();
        detectAngles.Dispose();
        avoidRadii.Dispose();
        alignWeights.Dispose();
        cohesionWeights.Dispose();
        separateWeights.Dispose();
        steerForces.Dispose();
        maxSpeeds.Dispose();
        accelerations.Dispose();
    }

    void Start()
    {
        if (settings == null || prefab == null)
        {
            Debug.Log("Error: No settings or prefab provided.");
            Destroy(this);
            return;
        }

        // Initialize entities[i] array.
        void CreateEntities()
        {
            entities = new SchoolBoid[spawnCount];
            for (var i = 0; i < entities.Length; i++)
            {
                var t = transforms[i];
                entities[i] = new()
                {
                    id = i,
                    position = t.position,
                    velocity = t.forward * UnityEngine.Random.Range(settings.minSpeed, settings.maxSpeed),
                    transform = t,
                    target = this.transform,
                    checkCollision = settings.enableCollisions,
                    detectedNeighbors = 0
                };
            }
        }

        // Initialize transform array than copy spatial data from each transform into the entities[i]
        // array. Replace with a matrix to avoid GameObject overhead later?
        transforms = SchoolUtilities.InstantiateEntityTransforms(prefab, this.transform.position, spawnRange, spawnCount);
        CreateEntities();
        SetupMeshInstancing();
        AllocateJobContainers();
    }

    void Update()
    {
        // Don't update calculations if there is a lag spike (<10FPS for now), since it may cause
        // teleportation through colliders.
        if (Time.deltaTime < 0.1f)
        {
            UpdateEntityVelocitiesParallel();
        }
        MoveEntities();
        UpdateTransforms();
        if (settings.useMeshInstancing)
        {
            if (instanceMaterial == null || instanceMesh == null)
                return;
            SchoolUtilities.RenderInstances(transforms, instanceMesh, instanceMaterial);
        }
        frameCount += 1;
    }

    /// <summary>
    /// Apply acceleration to an entity's velocity, then reset its acceleration.
    /// TODO: Remove the acceleration field from the struct and change it to a paramter.
    /// </summary>
    /// <param name="i">The index of the entity.</param>
    void ApplyAcceleration(int i)
    {
        if (math.length(entities[i].velocity) == 0)
        {
            entities[i].velocity = Unity.Mathematics.Random.CreateFromIndex(0).NextFloat3Direction() * settings.minSpeed;
        }
        var newVelocity = entities[i].velocity + entities[i].acceleration * Time.deltaTime;
        var turnAmount = math.degrees(math.acos(math.dot(entities[i].velocity, newVelocity)));
        if (turnAmount > settings.maxTurnSpeed * Time.deltaTime)
        {
            newVelocity = math.lerp(entities[i].velocity, newVelocity, settings.maxTurnSpeed * Time.deltaTime);
        }
        entities[i].velocity = newVelocity;
        var speed = math.clamp(math.length(entities[i].velocity), settings.minSpeed, settings.maxSpeed);
        entities[i].velocity = speed * math.normalize(entities[i].velocity);
        entities[i].acceleration = new();
        // entities[i].forward = math.normalize(entities[i].velocity);
    }

    /// <summary>
    /// Compute and apply collision avoidance force for a given entity automatically.
    /// </summary>
    /// <param name="i">The index of the entity.</param>
    void ApplyCollisionAvoidance(int i)
    {
        var position = entities[i].position;
        var velocity = entities[i].velocity;
        var weight = settings.avoidCollisionWeight * (settings.skipCollisionFrames ? settings.collisionFrameSkips : 1);
        var dist = settings.collisionCheckDistance;
        var radius = settings.collisionCheckRadius;
        var layers = settings.collisionMask;
        var steerForce = steerForces[i];
        var maxSpeed = maxSpeeds[i];

        // TODO: Change this to be a new float3 later and remove acceleration property
        // from entity (?) since its computed per-frame.
        var acceleration = entities[i].acceleration;
        acceleration += SchoolUtilities.AvoidCollisionForce(position, velocity, weight, dist, radius, layers, steerForce, maxSpeed);
        entities[i].acceleration = acceleration;
    }

    /// <summary>
    /// Reset values of the temporary data in the entity which is recalculated every frame.
    /// </summary>
    /// <param name="i">The index of the entity.</param>
    void ResetEntityTempData(int i)
    {
        entities[i].detectedNeighbors = 0;
        entities[i].neighborHeading = new();
        entities[i].neighborCenter = new();
        entities[i].avoidHeading = new();
        entities[i].acceleration = new();
    }

    void UpdateJobsContainers()
    {
        var size = entities.Length;
        for (int i = 0; i < size; i++)
        {
            positions[i] = entities[i].position;
            velocities[i] = entities[i].velocity;
            avoidRadii[i] = settings.avoidanceRadius;
            cohesionWeights[i] = settings.cohesionWeight;
            alignWeights[i] = settings.alignWeight;
            cohesionWeights[i] = settings.cohesionWeight;
            separateWeights[i] = settings.separateWeight;
            steerForces[i] = settings.maxSteerForce;
            maxSpeeds[i] = settings.maxSpeed;
        }
    }

    void UpdateEntityVelocitiesParallel()
    {
        UpdateJobsContainers();
        // Create and run the job with a preset batch count.
        var accelerationsJob = new SchoolComputeAccelerationJob
        {
            positions = positions,
            velocities = velocities,
            detectRadii = detectRadii,
            detectAngles = detectAngles,
            avoidRadii = avoidRadii,
            alignWeights = alignWeights,
            cohesionWeights = cohesionWeights,
            separateWeights = separateWeights,
            steerForces = steerForces,
            maxSpeeds = maxSpeeds,
            accelerations = accelerations
        };

        // FIXME: This can probably be higher (8 seems good).
        var handle = accelerationsJob.Schedule(entities.Length, settings.parallelJobBatchCount);
        // Once the job is completed, apply accelerations to each entity then dispose of data.
        handle.Complete();
        for (var i = 0; i < entities.Length; i++)
        {
            var acceleration = accelerations[i];
            var steerForce = steerForces[i];
            var maxSpeed = maxSpeeds[i];
            if (entities[i].target != null)
            {
                var targetOffset = new float3(entities[i].target.position) - entities[i].position;
                var targetForce = settings.targetWeight * SchoolMath.SteerTowards(
                    velocities[i], targetOffset, steerForce * 1000, maxSpeed);
                acceleration += targetForce;
            }
            entities[i].acceleration = acceleration;
            // ApplyAcceleration(i);

            // Check if collisions should be calculated, skip RayCast if unneeded.
            if (!settings.enableCollisions)
            {
                ApplyAcceleration(i);
                continue;
            }
            else if (settings.skipCollisionFrames && (i + frameCount) % settings.collisionFrameSkips != 0)
            {
                ApplyAcceleration(i);
                continue;
            }
            // Collision checking is enabled, so compute and apply the collision force.
            ApplyCollisionAvoidance(i);
            ApplyAcceleration(i);
        }
    }

    /// <summary>
    /// Update the position and heading of each velocity. Assumes velocity.length > 0.
    /// </summary>
    void MoveEntities()
    {
        for (var i = 0; i < entities.Length; i++)
        {
            var nextPos = entities[i].position + entities[i].velocity * Time.deltaTime;
            entities[i].position = nextPos;
        }
    }

    /// <summary>
    /// Apply the updated entities[i] data onto the corresponding GameObjects.
    /// TODO: Preferably replace this with matrices later to reduce overhead when moving to a
    /// DOTS-based system.
    /// </summary>
    void UpdateTransforms()
    {
        for (var i = 0; i < transforms.Length; i++)
        {
            transforms[i].position = entities[i].position;
            transforms[i].forward = entities[i].forward;
        }
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = new(Color.cyan.r, Color.cyan.g, Color.cyan.b, 0.1f);
        Gizmos.DrawSphere(transform.position, spawnRange.x);
        Gizmos.DrawSphere(transform.position, spawnRange.y);
    }
}