using UnityEngine;
using UnityEngine.InputSystem;

namespace HutsStudio.ThirdPersonController
{
    public class CursorManager : MonoBehaviour
    {
    [Header("Cursor Settings")]
    [Tooltip("Locks and hides the cursor automatically when the game starts.")]
    public bool lockOnStart = true;

    private void Start()
    {
        if (lockOnStart)
        {
            LockCursor();
        }
    }

    public void OnUnlockCursor(InputAction.CallbackContext context)
    {
        if (context.performed)
        {
            UnlockCursor();
        }
    }

    public void OnLockCursor(InputAction.CallbackContext context)
    {
        if (context.performed && Cursor.lockState != CursorLockMode.Locked)
        {
            LockCursor();
        }
    }

    public void LockCursor()
    {
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    public void UnlockCursor()
    {
        Cursor.lockState = CursorLockMode.None;
        Cursor.visible = true;
    }
}
}