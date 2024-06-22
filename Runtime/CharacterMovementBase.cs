using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace CharacterMovement
{
    public abstract class CharacterMovementBase : MonoBehaviour
    {
        // private serialized fields
        [field: Header("Movement")]
        [field: SerializeField] public float Speed { get; set; } = 5f;
        [field: SerializeField] public float Acceleration { get; set; } = 10f;
        [field: SerializeField] public float TurnSpeed { get; set; } = 15f;
        [field: SerializeField] public float StoppingDistance { get; set; } = 0.25f;
        [field: SerializeField] public bool LookInMoveDirection { get; set; } = true;
        [field: SerializeField] public bool ControlRotation { get; set; } = true;       // character turns towards movement direction
        [field: SerializeField] public bool Fix3DSpriteRotation { get; set; } = false;
        [field: SerializeField] public bool ParentToSurface { get; set; } = false;

        [field: Header("Airborne")]
        [field: SerializeField] public float Gravity { get; set; } = -20f;             // custom gravity value
        [field: SerializeField] public float JumpHeight { get; set; } = 2.25f;         // peak height of jump
        [field: SerializeField] public float AirControl { get; set; } = 0.1f;          // percentage of acceleration applied while airborne
        [field: SerializeField] public bool AirTurning { get; set; } = true;           // character can turn while airborne

        [field: Header("Size")]
        [field: SerializeField] public float Height { get; protected set; } = 1.8f;
        [field: SerializeField] public float Radius { get; protected set; } = 0.3f;

        [field: Header("Grounding")]
        [field: SerializeField] protected float GroundCheckOffset { get; set; } = 0.1f;         // height inside character where grounding ray starts
        [field: SerializeField] protected float GroundCheckDistance { get; set; } = 0.4f;       // distance down from offset position
        [field: SerializeField] protected float MaxSlopeAngle { get; set; } = 40f;              // maximum climbable slope, character will slip on anything higher
        [field: SerializeField] protected float CoyoteMaxJumpDistance { get; set; } = 0.5f;     // max distance allowed after leaving ground when doing a coyote jump
        [field: SerializeField] protected LayerMask GroundMask { get; set; } = 1 << 0;          // mask for layers considered the ground

        [field: Header("Events")]
        public UnityEvent<GameObject> OnGrounded;
        public UnityEvent<GameObject> OnFootstep;

        // public properties
        public float MoveSpeedMultiplier { get; set; } = 1f;
        public bool CanCoyoteJump => LastGroundedDistance < CoyoteMaxJumpDistance;
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
        public bool CanTurn { get; set; } = true;
        public Vector3 GroundNormal { get; protected set; } = Vector3.up;
        public float LastGroundedTime { get; protected set; }
        public Vector3 LastGroundedPosition { get; protected set; }

        // methods
        public virtual void Jump() { }
        public virtual void SetMoveInput(Vector3 input) { }
        public virtual void SetLookDirection(Vector3 direction) { }
        public virtual void SetLookPosition(Vector3 position) { }
        public virtual void FootstepAnimEvent(AnimationEvent animationEvent)
        {
            if (animationEvent.animatorClipInfo.weight > 0.5f && IsGrounded && NormalizedSpeed > 0.05f) OnFootstep.Invoke(SurfaceObject);
        }
    }
}