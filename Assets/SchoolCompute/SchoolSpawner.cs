using UnityEngine;

public enum GizmoDrawMode
{
    Never,
    Selected,
    Always
}

public class SchoolSpawner : MonoBehaviour
{
    public SchoolEntity entityPrefab;
    public Vector2 spawnOffsetRange = new(8, 10);
    public int spawnCount = 100;

    public Color colour;
    public GizmoDrawMode showSpawnRegion;

    void Awake()
    {
        for (int i = 0; i < spawnCount; i++)
        {
            var posOffset = Random.onUnitSphere * Random.Range(spawnOffsetRange.x, spawnOffsetRange.y);
            var fwd = new Vector3(posOffset.z, posOffset.y, posOffset.x);
            var rot = Quaternion.LookRotation(fwd, Vector3.up);
            var entity = Instantiate(entityPrefab, transform.position + posOffset, rot);
        }
    }

    private void OnDrawGizmos()
    {
        if (showSpawnRegion == GizmoDrawMode.Always)
            DrawGizmos();
    }

    void OnDrawGizmosSelected()
    {
        if (showSpawnRegion == GizmoDrawMode.Selected)
            DrawGizmos();
    }

    void DrawGizmos()
    {
        Gizmos.color = new(colour.r, colour.g, colour.b, 0.3f);
        Gizmos.DrawSphere(transform.position, spawnOffsetRange.x);
        Gizmos.DrawSphere(transform.position, spawnOffsetRange.y);
    }
}