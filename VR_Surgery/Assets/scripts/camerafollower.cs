using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target;  // The object to follow
    public Vector3 offset = new Vector3(0, 5, -10);  // Offset from the target
    public float smoothSpeed = 5f;  // Speed of smoothing

    void LateUpdate()
    {
        if (target == null) return; // Avoid errors if no target is set

        // Desired position of the camera
        Vector3 desiredPosition = target.position + offset;

        // Smoothly move the camera to the desired position
        transform.position = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed * Time.deltaTime);

        // Optional: Make the camera look at the target
        transform.LookAt(target);
    }
}
