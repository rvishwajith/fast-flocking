using UnityEngine;
using UnityEngine.Rendering;
using Unity.Mathematics;

public class School : MonoBehaviour
{
    [SerializeField] SchoolSettings settings;
    SchoolEntity[] entities;

    Mesh instanceMesh;
    Material instanceMaterial;

    /// <summary>
    /// Compute shader thread group size.
    /// </summary>
    const int threadGroupSize = 1024;
    int frameCount = 0;

    [SerializeField] ComputeShader computeShader = null;
    [SerializeField] bool passCompute = true;

    void Start()
    {
        if (settings == null)
            return;

        void InitEntitites()
        {
            entities = FindObjectsByType<SchoolEntity>(FindObjectsSortMode.InstanceID);
            foreach (var entity in entities)
                entity.Initialize(settings, null);
        }
        InitEntitites();
        if (entities.Length == 0)
            return;

        if (settings.useMeshInstancing)
        {
            instanceMesh = entities[0].GetComponent<MeshFilter>().mesh;
            instanceMaterial = entities[0].GetComponent<MeshRenderer>().material;
            for (var i = 0; i < entities.Length; i++)
                entities[i].GetComponent<MeshRenderer>().enabled = false;
            // Debug.Log("Entity mesh instance: " + instanceMesh);
        }
    }

    void Update()
    {
        if (entities == null)
            return;

        void UpdateCollisionSettings()
        {
            // Update the transform of each entity, skipping collisions based on the frame counter if
            // enabled in settings.
            for (var i = 0; i < entities.Length; i++)
            {
                var enableCollisions = false;
                if (settings.enableCollisions)
                {
                    if (!settings.skipCollisionFrames)
                        enableCollisions = true;
                    else if ((i + frameCount) % settings.collisionFrameSkips == 0)
                        enableCollisions = true;
                }
                entities[i].enableCollisions = enableCollisions;
                entities[i].UpdateData();
            }
        }

        // Compute the forces of each entity.
        ComputeForces();
        UpdateCollisionSettings();

        // Render meshes if using GPU instancing.
        if (settings.useMeshInstancing)
            DrawInstancedMeshes();
        frameCount += 1;
    }

    void ComputeForces()
    {
        if (computeShader == null)
            return;

        SchoolEntityComputeData[] GetComputeBufferData()
        {
            var entityData = new SchoolEntityComputeData[entities.Length];
            for (int i = 0; i < entities.Length; i++)
            {
                entityData[i].position = entities[i].position;
                entityData[i].direction = entities[i].forward;
            }
            return entityData;
        }

        var entityData = GetComputeBufferData();
        var computeBuffer = new ComputeBuffer(entities.Length, SchoolEntityComputeData.Size, ComputeBufferType.Default, ComputeBufferMode.Dynamic);

        void PassComputeShaderData(ComputeShader computeShader)
        {
            computeBuffer.SetData(GetComputeBufferData());
            computeShader.SetBuffer(0, "entities", computeBuffer);
            computeShader.SetInt("entityCount", entities.Length);
            computeShader.SetFloat("perceptionRadius", settings.perceptionRadius);
            computeShader.SetFloat("avoidRadius", settings.avoidanceRadius);
        }

        void RunComputeShader(ComputeShader computeShader)
        {
            // FIXME: Dispatching and waiting for the GPU to finish is very slow. Look into async GPU
            // data requests.
            int threadGroups = Mathf.CeilToInt(entities.Length / (float)threadGroupSize);
            computeShader.Dispatch(0, threadGroups, 1, 1);
        }

        if (passCompute)
        {
            PassComputeShaderData(computeShader);
            RunComputeShader(computeShader);
            // Compute buffer is the output buffer.
            Debug.Log("Started GPU request on frame " + frameCount);
            var request = AsyncGPUReadback.Request(computeBuffer, OnComputeShaderReadback);
            request.forcePlayerLoopUpdate = true;
            // computeBuffer.GetData(entityData);
            passCompute = false;
        }

        void CopyDataBack()
        {
            for (int i = 0; i < entities.Length; i++)
            {
                entities[i].avgFlockHeading = entityData[i].flockHeading;
                entities[i].avgNeighborPosition = entityData[i].flockCenter;
                entities[i].avgAvoidanceHeading = entityData[i].avoidanceHeading;
                entities[i].percievedNeighborCount = entityData[i].neighborCount;
            }
        }

        void OnComputeShaderReadback(AsyncGPUReadbackRequest request)
        {
            if (!request.done)
            {
                passCompute = false;
                return;
            }
            else if (request.hasError)
            {
                Debug.Log("School: AsyncGPUReadbackRequest has error!");
                return;
            }
            // Debug.Log("Completed readback request on frame " + frameCount);
            // NOTE: This copies data from the compute buffer INTO entityData.
            computeBuffer.GetData(entityData);
            CopyDataBack();
            computeBuffer.Release();
            passCompute = true;
        }
    }

    void DrawInstancedMeshes()
    {
        if (instanceMesh == null || instanceMaterial == null)
            return;
        // Render parameters are the same for all isntances.
        var renderParams = new RenderParams(instanceMaterial);
        var meshesRendered = 0;
        var instanceStep = SchoolMath.MAX_INSTANCE_BATCH_SIZE;
        for (var i = 0; i < entities.Length; i += instanceStep)
        {
            var instanceData = new Matrix4x4[Mathf.Min(i + instanceStep, entities.Length) - i];
            for (var j = 0; j < instanceData.Length; j++)
            {
                if (i + j >= entities.Length)
                {
                    Debug.Log("entityI is too large: " + (i + j));
                    continue;
                }
                instanceData[j] = entities[i + j].transform.localToWorldMatrix;
                meshesRendered += 1;
            }
            Graphics.RenderMeshInstanced(renderParams, instanceMesh, 0, instanceData);
        }
        // Debug.Log("Meshes rendered: " + meshesRendered);
    }
}

public struct SchoolEntityComputeData
{
    public float3 position;
    public float3 direction;
    public float3 flockHeading;
    public float3 flockCenter;
    public float3 avoidanceHeading;
    public int neighborCount;

    // 5 Vector3s + 1 int
    public static int Size { get { return 5 * (3 * sizeof(float)) + sizeof(int); } }
}