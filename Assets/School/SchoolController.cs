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

    /// <summary>
    /// Array of entities representing a single fish. Stores values for position, velocity, and
    /// will later also store weight data, etc.
    /// </summary>
    SchoolBoid[] entities;

    int frameCount = 0; // Remove this later.

    // Use GPU instancing if enabled in settings.
    Mesh instanceMesh;
    Material instanceMaterial;

    // Native arrays for jobs.
    NativeArray<float3> positions;
    NativeArray<float3> velocities;
    NativeArray<float> detectRadii;
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

    void AllocateJobData()
    {
        var size = entities.Length;
        var allocator = Allocator.Persistent;
        positions = new NativeArray<float3>(size, allocator);
        velocities = new NativeArray<float3>(size, allocator);
        for (int i = 0; i < size; i++)
        {
            positions[i] = entities[i].position;
            velocities[i] = entities[i].velocity;
        }
        detectRadii = SchoolMath.ToNativeArray(settings.perceptionRadius, size, allocator);
        avoidRadii = SchoolMath.ToNativeArray(settings.avoidanceRadius, size, allocator);
        alignWeights = SchoolMath.ToNativeArray(settings.alignWeight, size, allocator);
        cohesionWeights = SchoolMath.ToNativeArray(settings.cohesionWeight, size, allocator);
        separateWeights = SchoolMath.ToNativeArray(settings.separateWeight, size, allocator);
        steerForces = SchoolMath.ToNativeArray(settings.maxSteerForce, size, allocator);
        maxSpeeds = SchoolMath.ToNativeArray(settings.maxSpeed, size, allocator);
        accelerations = new NativeArray<float3>(size, allocator);
    }

    void Start()
    {
        if (settings == null || prefab == null)
        {
            Debug.Log("Error: No settings or prefab provided.");
            Destroy(this);
            return;
        }

        // Initialize entities[i] transform array.
        Transform[] InstantiateEntityTransforms()
        {
            var transforms = new Transform[spawnCount];
            for (var i = 0; i < transforms.Length; i++)
            {
                var t = Instantiate(prefab).transform;
                t.name = "Entity" + i;
                var offsetDist = UnityEngine.Random.Range(spawnRange.x, spawnRange.y);
                t.position = this.transform.position + UnityEngine.Random.onUnitSphere * offsetDist;
                t.LookAt(this.transform.position, Vector3.up);
                t.forward = t.right;
                transforms[i] = t;
            }
            return transforms;
        }

        // Initialize entities[i] array.
        void CreateEntities()
        {
            var transforms = InstantiateEntityTransforms();
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
        CreateEntities();
        SetupMeshInstancing();
        if (settings.enableParallelJobs)
            AllocateJobData();
    }

    void Update()
    {
        // Don't update calculations if there is a lag spike (<10FPS for now), since it may cause
        // teleportation through colliders.
        if (Time.deltaTime < 0.1f)
        {
            if (settings.enableParallelJobs)
                UpdateEntityVelocitiesParallel();
            else
                UpdateEntityVelocities();
            MoveEntities();
        }
        UpdateTransforms();
        RenderInstances();
        frameCount += 1;
    }

    /// <summary>
    /// Apply acceleration to an entity's velocity, then reset its acceleration.
    /// TODO: Remove the acceleration field from the struct and change it to a paramter.
    /// </summary>
    /// <param name="i">Index of the entity to reset.</param>
    void ApplyAcceleration(int i)
    {
        entities[i].velocity += entities[i].acceleration * Time.deltaTime;
        if (math.length(entities[i].velocity) == 0)
        {
            entities[i].velocity = Unity.Mathematics.Random.CreateFromIndex(0).NextFloat3Direction();
        }
        var speed = math.clamp(math.length(entities[i].velocity), settings.minSpeed, settings.maxSpeed);
        entities[i].velocity = speed * math.normalize(entities[i].velocity);
        entities[i].acceleration = new();
        // entities[i].forward = math.normalize(entities[i].velocity);
    }

    /// <summary>
    /// Reset values of the temporary data in the entity which is recalculated every frame.
    /// </summary>
    /// <param name="i">Index of the entity to reset.</param>
    void ResetEntityTempData(int i)
    {
        entities[i].detectedNeighbors = 0;
        entities[i].neighborHeading = new();
        entities[i].neighborCenter = new();
        entities[i].avoidHeading = new();
        entities[i].acceleration = new();
    }

    /// <summary>
    /// Compute a collision avoidance vector for each entity. Assumes that collision is enabled
    /// for the entity.
    /// </summary>
    /// <param name="i">Index of the entity to compute the vector for.</param>
    /// <returns></returns>
    float3 ComputeCollisionAvoidance(int i)
    {
        var collisionWeight = settings.avoidCollisionWeight;
        var castDist = settings.collisionCheckDistance;
        var layers = settings.collisionMask;
        var steerForce = settings.maxSteerForce;
        var velocity = entities[i].velocity;
        var radius = settings.collisionCheckRadius;
        var pos = entities[i].position;
        // entities[i].transform.forward = entities[i].velocity;
        var rot = quaternion.LookRotation(math.normalize(velocity), SchoolMath.WORLD_UP);
        var dirs = SchoolMath.TURN_DIRS_MED;
        for (int k = 0; k < dirs.Length; k++)
        {
            // var dir = entities[i].transform.TransformDirection(dirs[k]);
            var dir = math.rotate(rot, dirs[k]);
            var ray = new Ray(pos, dir);
            if (radius > 0 && !Physics.SphereCast(ray, radius, castDist, layers))
            {
                return collisionWeight * SchoolMath.SteerTowards(
                    velocity, dir, steerForce, settings.maxSpeed);
            }
            else if (radius == 0 && !Physics.Raycast(ray, castDist, layers))
            {
                return collisionWeight * SchoolMath.SteerTowards(
                    velocity, dir, steerForce, settings.maxSpeed);
            }
        }
        return new float3();
    }

    void UpdateEntityVelocities()
    {
        // Compute acceleration for all entities.
        for (int i = 0; i < entities.Length; i++)
        {
            var detectRadius = settings.perceptionRadius;
            var avoidRadius = settings.avoidanceRadius;

            var velocity = entities[i].velocity;
            var maxSpeed = settings.maxSpeed;
            var steerForce = settings.maxSteerForce;

            // Reset all values.
            ResetEntityTempData(i);

            // Complete neighbor-based calculations.
            // FIXME: Currently this is slow, possible optimizations:
            // 1. Add colliders to transforms and use Physics.OverlapSphere()?
            // 2. Use jobs or a compute shader?
            for (int j = 0; j < entities.Length; j++)
            {
                if (i == j)
                    continue;
                var neighbor = entities[j];
                float dist = math.distance(neighbor.position, entities[i].position);
                if (dist > 0 && dist <= detectRadius)
                {
                    entities[i].detectedNeighbors += 1;
                    entities[i].neighborHeading += neighbor.forward;
                    entities[i].neighborCenter += neighbor.position;
                    if (dist <= avoidRadius)
                        entities[i].avoidHeading += (entities[i].position - neighbor.position) / (dist * dist);
                }
            }

            // If the entity has detected neighbors, update the acceleration for it.
            if (entities[i].detectedNeighbors != 0)
            {
                entities[i].neighborCenter /= entities[i].detectedNeighbors;
                // Compute align force.
                var align = settings.alignWeight * SchoolMath.SteerTowards(
                    velocity, entities[i].neighborHeading, steerForce, maxSpeed);
                // Compute center of mass (attraction) force.
                var toCenter = settings.cohesionWeight * SchoolMath.SteerTowards(
                    velocity, entities[i].neighborCenter - entities[i].position, steerForce, maxSpeed);
                // Compute separate (move away from neighbor position) force.
                var separate = settings.separateWeight * SchoolMath.SteerTowards(
                    velocity, entities[i].avoidHeading, steerForce, maxSpeed);
                // Apply forces to acceleration.
                entities[i].acceleration += align + toCenter + separate;
            }

            // Compute target attraction force if a target is set.
            if (entities[i].target != null)
            {
                var targetOffset = new float3(entities[i].target.position) - entities[i].position;
                var targetForce = settings.targetWeight * SchoolMath.SteerTowards(
                    velocity, targetOffset, steerForce, maxSpeed);
                entities[i].acceleration += targetForce;
            }
            ApplyAcceleration(i);
        }

        // Compute acceleration for collision avoidance.
        for (int i = 0; i < entities.Length; i++)
        {
            // Check if collisions should be calculated, skip RayCast if unneeded.
            if (!settings.enableCollisions)
                continue;
            else if (settings.skipCollisionFrames && (i + frameCount) % settings.collisionFrameSkips != 0)
                continue;
            entities[i].acceleration += ComputeCollisionAvoidance(i);
            ApplyAcceleration(i);
        }
    }

    void UpdateEntityVelocitiesParallel()
    {
        // Create and populate input/output arrays for job data.
        var size = entities.Length;
        for (int i = 0; i < size; i++)
        {
            positions[i] = entities[i].position;
            velocities[i] = entities[i].velocity;
        }

        // Create and run the job with a preset batch count.
        var accelerationsJob = new SchoolComputeAccelerationJob
        {
            positions = positions,
            velocities = velocities,
            detectRadii = detectRadii,
            avoidRadii = avoidRadii,
            alignWeights = alignWeights,
            cohesionWeights = cohesionWeights,
            separateWeights = separateWeights,
            steerForces = steerForces,
            maxSpeeds = maxSpeeds,
            accelerations = accelerations
        };

        // FIXME: This can probably be higher (8 seems good).
        var batchCount = settings.parallelJobBatchCount;
        var handle = accelerationsJob.Schedule(size, batchCount);

        // Once the job is completed, apply accelerations to each entity then dispos of data.
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
                    velocities[i], targetOffset, steerForce, maxSpeed);
                acceleration += targetForce;
            }
            entities[i].acceleration = acceleration;
            ApplyAcceleration(i);

            // Check if collisions should be calculated, skip RayCast if unneeded.
            if (!settings.enableCollisions)
                continue;
            else if (settings.skipCollisionFrames && (i + frameCount) % settings.collisionFrameSkips != 0)
                continue;
            entities[i].acceleration += ComputeCollisionAvoidance(i);
            ApplyAcceleration(i);
        }
        // Debug.Log("Parallel acceleration computations completed, disposing of data.");
        // DisposeJobData();
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
        for (var i = 0; i < entities.Length; i++)
        {
            entities[i].transform.position = entities[i].position;
            entities[i].transform.forward = entities[i].forward;
        }
    }

    /// <summary>
    /// If mesh instancing is enabled, render the entities with the given mesh and material
    /// properties using Graphics.RenderMeshInstanced.
    /// </summary>
    void RenderInstances()
    {
        if (!settings.useMeshInstancing || instanceMesh == null || instanceMaterial == null)
            return;

        var renderParams = new RenderParams(instanceMaterial);
        var meshesRendered = 0;
        var step = SchoolMath.MAX_INSTANCE_BATCH_SIZE;
        for (var i = 0; i < entities.Length; i += step)
        {
            var numInstances = math.min(i + step, entities.Length) - i;
            // if (frameCount == 5)
            //     Debug.Log("Meshes to render: " + numInstances);
            var instanceMatrices = new Matrix4x4[numInstances];
            for (var j = 0; j < numInstances; j++)
            {
                instanceMatrices[j] = entities[i + j].transform.localToWorldMatrix;
                meshesRendered += 1;
            }
            Graphics.RenderMeshInstanced(renderParams, instanceMesh, 0, instanceMatrices);
        }
        // if (frameCount == 5)
        //     Debug.Log("Meshes rendered: " + meshesRendered);
    }

    void OnDestroy()
    {
        Debug.Log("Killing school controller, disposing of allocated jobs data.");
        positions.Dispose();
        velocities.Dispose();
        detectRadii.Dispose();
        avoidRadii.Dispose();
        alignWeights.Dispose();
        cohesionWeights.Dispose();
        separateWeights.Dispose();
        steerForces.Dispose();
        maxSpeeds.Dispose();
        accelerations.Dispose();
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = new(Color.cyan.r, Color.cyan.g, Color.cyan.b, 0.1f);
        Gizmos.DrawSphere(transform.position, spawnRange.x);
        Gizmos.DrawSphere(transform.position, spawnRange.y);
    }
}