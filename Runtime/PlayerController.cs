using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace CharacterMovement
{
    // sends input from PlayerInput to attached CharacterMovement components
    public class PlayerController : MonoBehaviour
    {
        // initial cursor state
        [SerializeField] protected CursorLockMode _cursorMode = CursorLockMode.Locked;
        // make character look in Camera direction instead of MoveDirection
        [SerializeField] protected bool _lookInCameraDirection;

        protected CharacterMovementBase _characterMovement;
        protected Vector2 _moveInput;

        protected virtual void Awake()
        {
            _characterMovement = GetComponent<CharacterMovementBase>();
            Cursor.lockState = _cursorMode;
        }

        public virtual void OnMove(InputValue value)
        {
            _moveInput = value.Get<Vector2>();
        }

        public virtual void OnJump(InputValue value)
        {
            _characterMovement?.Jump();
        }

        public virtual void OnFire(InputValue value)
        {
            // placeholder for shooting stuff
        }

        protected virtual void Update()
        {
            if (_characterMovement == null) return;

            // find correct right/forward directions based on main camera rotation
            Vector3 up = Vector3.up;
            Vector3 right = Camera.main.transform.right;
            Vector3 forward = Vector3.Cross(right, up);
            Vector3 moveInput = forward * _moveInput.y + right * _moveInput.x;

            // send player input to character movement
            _characterMovement.SetMoveInput(moveInput);
            _characterMovement.SetLookDirection(moveInput);
            if (_lookInCameraDirection) _characterMovement.SetLookDirection(Camera.main.transform.forward);
        }
    }
}