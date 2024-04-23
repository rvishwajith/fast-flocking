using UnityEngine;

class SetFrameRate : MonoBehaviour
{
    [SerializeField] int targetFrameRate = -1;

    public void FixedUpdate()
    {
        if (targetFrameRate != Application.targetFrameRate)
        {
            // Debug.Log("Changing frame rate to " + targetFrameRate);
            Application.targetFrameRate = targetFrameRate;
        }
    }
}