using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace CharacterMovement
{
    public abstract class CharacterMovementBase : MonoBehaviour
    {
        // private serialized fields
        [Header("Movement")]
        [SerializeField] protected float _speed = 5f;
        [SerializeField] protected float _acceleration = 10f;
        [SerializeField] protected float _turnSpeed = 10f;
        [SerializeField] protected float _stoppingDistance = 0.25f;
        [SerializeField] protected bool _controlRotation = true;    // character turns towards movement direction

        [Header("Airborne")]
        [SerializeField] protected float _gravity = -20f;       // custom gravity value
        [SerializeField] protected float _jumpHeight = 2.25f;   // peak height of jump
        [SerializeField] protected float _airControl = 0.1f;    // percentage of acceleration applied while airborne
        [SerializeField] protected bool _airTurning = true;     // character can turn while airborne

        [Header("Grounding")]
        [SerializeField] protected float _groundCheckOffset = 0.1f;         // height inside character where grounding ray starts
        [SerializeField] protected float _groundCheckDistance = 0.4f;       // distance down from offset position
        [SerializeField] protected float _maxSlopeAngle = 40f;              // maximum climbable slope, character will slip on anything higher
        [SerializeField] protected float _coyoteJumpMaxDistance = 0.5f;      // max distance allowed after leaving ground when doing a coyote jump
        [SerializeField] protected LayerMask _groundMask = 1 << 0;          // mask for layers considered the ground

        [Header("Events")]
        public UnityEvent<GameObject> OnGrounded;
        public UnityEvent<GameObject> OnFootstep;

        // public properties
        public float MoveSpeedMultiplier { get; set; } = 1f;
        public float Speed => _speed;
        public bool CanCoyoteJump => LastGroundedDistance < _coyoteJumpMaxDistance;
        public float LastGroundedDistance => Vector3.Distance(transform.position, LastGroundedPosition);

        // public-get protected-set properties
        public virtual Vector3 Velocity { get; protected set; }
        public Vector3 FlattenedVelocity => new Vector3(Velocity.x, 0f, Velocity.z);
        public float NormalizedSpeed => FlattenedVelocity.magnitude / Speed;
        public Vector3 MoveInput { get; protected set; }
        public Vector3 LocalMoveInput { get; protected set; }
        public Vector3 LookDirection { get; protected set; }
        public bool HasMoveInput { get; protected set; }
        public bool HasTurnInput { get; protected set; }
        public bool IsGrounded { get; protected set; }
        public GameObject SurfaceObject { get; protected set; }
        public Vector3 SurfaceVelocity { get; protected set; }
        public bool CanMove { get; set; } = true;
        public Vector3 GroundNormal { get; protected set; } = Vector3.up;
        public float LastGroundedTime { get; protected set; }
        public Vector3 LastGroundedPosition { get; protected set; }

        // methods
        public virtual void Jump() { }
        public virtual void SetMoveInput(Vector3 input) { }
        public virtual void SetLookDirection(Vector3 direction) { }
        public virtual void FootstepAnimEvent(AnimationEvent animationEvent)
        {
            if (animationEvent.animatorClipInfo.weight > 0.5f && IsGrounded && NormalizedSpeed > 0.05f) OnFootstep.Invoke(SurfaceObject);
        }
    }
}