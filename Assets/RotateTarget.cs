using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateTarget : MonoBehaviour
{
    float rotation = 0;

    void Update()
    {
        rotation += 90 * Time.deltaTime;
        transform.rotation = Quaternion.Euler(Vector3.up * rotation);
    }
}
