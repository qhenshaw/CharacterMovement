using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CharacterMovement
{
    [RequireComponent(typeof(Rigidbody2D))]
    public class CharacterMovement2D : CharacterMovementBase
    {
        [field: Header("Top Down")]
        [field: SerializeField] protected bool TopDownMovement = false;

        [field: Header("Components")]
        [field: SerializeField] protected Rigidbody2D Rigidbody;
        [field: SerializeField] protected CapsuleCollider2D CapsuleCollider;

        public override Vector3 Velocity { get => Rigidbody.velocity; protected set => Rigidbody.velocity = value; }
        protected Vector3 GroundCheckStart => transform.position + transform.up * GroundCheckOffset;

        protected virtual void OnValidate()
        {
            if (Rigidbody == null) Rigidbody = GetComponent<Rigidbody2D>();
            Rigidbody.gravityScale = 0f;
            Rigidbody.freezeRotation = true;

            if (CapsuleCollider == null) CapsuleCollider = GetComponentInChildren<CapsuleCollider2D>();
            if (CapsuleCollider != null)
            {
                CapsuleCollider.size = new Vector2(Radius, Height);
                CapsuleCollider.offset = new Vector2(0f, Height * 0.5f);
            }
        }

        protected virtual void Awake()
        {
            if (CapsuleCollider != null)
            {
                CapsuleCollider.sharedMaterial = new PhysicsMaterial2D("NoFriction") { friction = 0f, bounciness = 0f };
                CapsuleCollider.size = new Vector2(Radius, Height);
                CapsuleCollider.offset = new Vector2(0f, Height * 0.5f);
            }

            LookDirection = Vector3.right;
        }

        // receives movement input and clamps it to prevent over-acceleration
        public override void SetMoveInput(Vector3 input)
        {
            if (!CanMove)
            {
                MoveInput = Vector3.zero;
                return;
            }

            input = Vector3.ClampMagnitude(input, 1f);
            // set input to 0 if small incoming value
            HasMoveInput = input.magnitude > 0.1f;
            input = HasMoveInput ? input : Vector3.zero;
            MoveInput = input;
            // finds movement input as local direction rather than world
            LocalMoveInput = transform.InverseTransformDirection(MoveInput);
        }

        // sets character look direction, flattening y-value
        public override void SetLookDirection(Vector3 direction)
        {
            if (!CanTurn || direction.magnitude < 0.1f) return;
            LookDirection = new Vector3(direction.x, 0f, direction.z).normalized;
        }

        public override void SetLookPosition(Vector3 position)
        {
            Vector3 direction = Vector3.ClampMagnitude(position - transform.position, 1f);
            SetLookDirection(direction);
        }

        // attempts a jump, will fail if not grounded
        public override void TryJump()
        {
            if (!CanMove || !CanCoyoteJump) return;
            Jump();
        }

        public override void Jump()
        {
            // calculate jump velocity from jump height and gravity
            float jumpVelocity = Mathf.Sqrt(2f * -Gravity * JumpHeight);
            // override current y velocity but maintain x/z velocity
            Velocity = new Vector3(Velocity.x, jumpVelocity, Velocity.z);
        }

        protected virtual void FixedUpdate()
        {
            // check for the ground
            IsGrounded = CheckGrounded();

            // sends correct forward/right inputs to GetMovementAcceleration and applies result to rigidbody
            Vector3 input = MoveInput;
            Vector3 forward = Vector3.right * input.x;
            if (TopDownMovement) forward = new Vector3(MoveInput.x, MoveInput.z, 0f);

            // calculates desirection movement velocity
            Vector3 targetVelocity = forward * (Speed * MoveSpeedMultiplier);
            if (!CanMove) targetVelocity = Vector3.zero;
            // adds velocity of surface under character, if character is stationary
            targetVelocity += SurfaceVelocity * (1f - Mathf.Abs(MoveInput.magnitude));
            // calculates acceleration required to reach desired velocity and applies air control if not grounded
            Vector3 velocityDiff = targetVelocity - Velocity;
            if (!TopDownMovement) velocityDiff.y = 0f;
            float control = IsGrounded ? 1f : AirControl;
            Vector3 acceleration = velocityDiff * (Acceleration * control);
            // zeros acceleration if airborne and not trying to move (allows for nice jumping arcs)
            if (!IsGrounded && !HasMoveInput) acceleration = Vector3.zero;
            // add gravity
            acceleration += GroundNormal * Gravity;

            Rigidbody.AddForce(acceleration);

            // rotates character towards movement direction
            if (ControlRotation && (IsGrounded || AirTurning))
            {
                transform.rotation = Quaternion.LookRotation(LookDirection);
            }

            // fix capsule collider rotation
            CapsuleCollider.transform.rotation = Quaternion.identity;
        }

        protected virtual bool CheckGrounded()
        {
            // ignore ground checks if top-down
            if (TopDownMovement) return true;

            // configure layer mask for 2D raycast
            ContactFilter2D filter = new ContactFilter2D();
            filter.SetLayerMask(GroundMask);
            // raycast to find ground
            RaycastHit2D[] hits = new RaycastHit2D[4];
            RaycastHit2D groundHit = new RaycastHit2D();
            Physics2D.Raycast(GroundCheckStart, -transform.up, filter, hits, GroundCheckDistance);
            for (int i = 0; i < hits.Length; i++)
            {
                RaycastHit2D hit = hits[i];
                if(hit.collider != null && hit.collider != CapsuleCollider)
                {
                    groundHit = hit;
                    continue;
                }
            }

            // set default ground surface normal and SurfaceVelocity
            GroundNormal = Vector3.up;
            SurfaceVelocity = Vector3.zero;

            // if ground wasn't hit, character is not grounded
            if (groundHit.collider == null) return false;

            // gets velocity of surface underneath character if applicable
            if (groundHit.rigidbody != null) SurfaceVelocity = groundHit.rigidbody.velocity;

            // test angle between character up and ground, angles above _maxSlopeAngle are invalid
            bool angleValid = Vector3.Angle(transform.up, groundHit.normal) < MaxSlopeAngle;
            if (angleValid)
            {
                // record last time character was grounded and set correct floor normal direction
                LastGroundedTime = Time.timeSinceLevelLoad;
                GroundNormal = groundHit.normal;
                LastGroundedPosition = transform.position;
                SurfaceObject = groundHit.collider.gameObject;
                if (ParentToSurface) transform.SetParent(SurfaceObject.transform);
                return true;
            }

            SurfaceObject = null;
            if (ParentToSurface) transform.SetParent(null);
            return false;
        }

        // check for landing on the ground
        private void OnCollisionEnter2D(Collision2D collision)
        {
            float landingCollisionMaxDistance = 0.25f;
            Vector3 point = collision.contacts[0].point;
            if (Mathf.Abs(collision.relativeVelocity.y) < MinGroundedVelocity) return;
            if (Vector3.Distance(point, transform.position) < landingCollisionMaxDistance)
            {
                OnGrounded.Invoke(collision.gameObject);
            }
        }

        protected virtual void OnDrawGizmosSelected()
        {
            Gizmos.color = IsGrounded ? Color.green : Color.red;
            Gizmos.DrawRay(GroundCheckStart, -transform.up * GroundCheckDistance);
        }
    }
}