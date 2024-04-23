// SchoolCreateOctreeJob.cs
// Author: Rohith Vishwajith
// Created 4/22/2024

using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;

public struct SchoolComputeOctreeJob : IJob
{
    // Inputs:
    // - positions[]: Positions of all entities.
    // - cellSize: The cell size of the octree.
    [ReadOnly] public NativeArray<float3> positions;
    [ReadOnly] public int cellSize;

    // Output:
    // - An octree with the keys as cell positions and values as lists of entity indices.
    [WriteOnly] public NativeParallelHashMap<int3, NativeList<int>> octree;

    public void Execute()
    {
        for (int i = 0; i < positions.Length; i++)
        {
            var key = GetKey(positions[i]);
            if (!octree.ContainsKey(key))
                octree[key] = new(Allocator.Temp);
            octree[key].Add(i);
        }
    }

    int3 GetKey(float3 position)
    {
        return new int3(cellSize * (math.floor(position) / cellSize));
    }
}