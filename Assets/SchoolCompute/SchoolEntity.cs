using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class SchoolEntity : MonoBehaviour
{
    SchoolSettings settings;

    // State
    [HideInInspector] public Vector3 position;
    [HideInInspector] public Vector3 forward;
    [HideInInspector] public Vector3 velocity;

    // To update:
    [HideInInspector] public Vector3 acceleration;
    [HideInInspector] public Vector3 avgFlockHeading;
    [HideInInspector] public Vector3 avgAvoidanceHeading;
    [HideInInspector] public Vector3 avgNeighborPosition;
    [HideInInspector] public int percievedNeighborCount;

    [HideInInspector] public bool enableCollisions = false;

    [HideInInspector] public Transform cachedTransform;
    [HideInInspector] public Transform target;

    void Awake()
    {
        cachedTransform = transform;
    }

    public void Initialize(SchoolSettings settings, Transform target)
    {
        this.target = target;
        this.settings = settings;
        this.enableCollisions = settings.enableCollisions;

        position = cachedTransform.position;
        forward = cachedTransform.forward;

        float startSpeed = (settings.minSpeed + settings.maxSpeed) / 2;
        velocity = transform.forward * startSpeed;
    }

    public void UpdateData()
    {
        var acceleration = Vector3.zero;
        if (target != null)
        {
            var targetOffset = target.position - position;
            acceleration = SteerTowards(targetOffset, settings.maxSteerForce) * settings.targetWeight;
        }

        if (percievedNeighborCount != 0)
        {
            avgNeighborPosition /= percievedNeighborCount;
            var offsetToFlockmatesCentre = avgNeighborPosition - position;
            var alignmentForce = SteerTowards(avgFlockHeading, settings.maxSteerForce) * settings.alignWeight;
            var cohesionForce = SteerTowards(offsetToFlockmatesCentre, settings.maxSteerForce) * settings.cohesionWeight;
            var seperationForce = SteerTowards(avgAvoidanceHeading, settings.maxSteerForce) * settings.separateWeight;
            acceleration += alignmentForce + cohesionForce + seperationForce;
        }

        if (enableCollisions && IsHeadingForCollision(position, forward))
            acceleration += ComputeCollisionForce();

        velocity += acceleration * Time.deltaTime;
        var currSpeed = velocity.magnitude;
        var dir = velocity / currSpeed;
        currSpeed = Mathf.Clamp(currSpeed, settings.minSpeed, settings.maxSpeed);
        velocity = dir * currSpeed;

        cachedTransform.position += velocity * Time.deltaTime;
        cachedTransform.forward = dir;
        position = cachedTransform.position;
        forward = dir;
    }

    Vector3 ComputeCollisionForce()
    {
        var collisionAvoidDir = AdjustedDirection();
        var collisionAvoidForce = SteerTowards(collisionAvoidDir, settings.maxSteerForce) * settings.avoidCollisionWeight;
        return collisionAvoidForce;
    }

    bool IsHeadingForCollision(Vector3 position, Vector3 direction)
    {
        RaycastHit hit;
        if (settings.collisionCheckRadius > 0)
            return Physics.SphereCast(position, settings.collisionCheckRadius, forward, out hit, settings.collisionCheckDistance, settings.collisionMask);
        else
            return Physics.Raycast(position, forward, settings.collisionCheckDistance, settings.collisionMask);
    }

    Vector3 AdjustedDirection()
    {
        var dirs = SchoolMath.TURN_DIRS_V3;
        for (int i = 0; i < dirs.Length; i++)
        {
            var dir = cachedTransform.TransformDirection(dirs[i]);
            var ray = new Ray(position, dir);
            var safeDir = false;
            if (settings.collisionCheckRadius > 0)
                safeDir = !Physics.SphereCast(ray, settings.collisionCheckRadius, settings.collisionCheckDistance, settings.collisionMask);
            else
                safeDir = !Physics.Raycast(ray, settings.collisionCheckDistance, settings.collisionMask);
            if (safeDir)
                return dir;
        }
        return forward;
    }

    Vector3 SteerTowards(Vector3 vector, float maxSteerForce)
    {
        var dir = vector.normalized * settings.maxSpeed - velocity;
        return Vector3.ClampMagnitude(dir, settings.maxSteerForce);
    }
}
