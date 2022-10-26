using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CharacterMovement
{
#pragma warning disable 649

    [RequireComponent(typeof(Rigidbody2D))]
    public class CharacterMovement2D : CharacterMovementBase
    {
        [Header("Collider should be child for 2.5D")]
        [SerializeField] private Collider2D _collider;

        [Header("Top Down")]
        [SerializeField] private bool _topDownMovement = false;

        // properties
        public override Vector3 Velocity { get => _rigidbody.velocity; protected set => _rigidbody.velocity = value; }
        private Vector3 _groundCheckStart => transform.position + transform.up * _groundCheckOffset;

        // private fields
        private Rigidbody2D _rigidbody;

        private void Start()
        {
            _rigidbody = GetComponent<Rigidbody2D>();
            _rigidbody.constraints = RigidbodyConstraints2D.FreezeRotation;
            _rigidbody.interpolation = RigidbodyInterpolation2D.Interpolate;
            _rigidbody.gravityScale = 0f;

            _collider = _collider ?? GetComponent<Collider2D>();
            PhysicsMaterial2D material = new PhysicsMaterial2D("NoFriction")
            {
                friction = 0f,
            };
            _collider.sharedMaterial = material;

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
            if (!CanMove || direction.magnitude < 0.1f) return;
            LookDirection = new Vector3(direction.x, 0f, direction.z).normalized;
        }

        // attempts a jump, will fail if not grounded
        public override void Jump()
        {
            if (!CanMove || !IsFudgeGrounded || LastGroundedDistance > _fudgeJumpMaxDistance) return;
            // calculate jump velocity from jump height and gravity
            float jumpVelocity = Mathf.Sqrt(2f * -_gravity * _jumpHeight);
            // override current y velocity but maintain x/z velocity
            Velocity = new Vector3(Velocity.x, jumpVelocity, Velocity.z);
        }

        // allows character to move and rotate
        public void SetCanMove(bool canMove)
        {
            CanMove = canMove;
        }

        private void FixedUpdate()
        {
            // sends correct forward/right inputs to GetMovementAcceleration and applies result to rigidbody
            Vector3 input = MoveInput;
            Vector3 forward = Vector3.right * input.x;
            if (_topDownMovement) forward = new Vector3(MoveInput.x, MoveInput.z, 0f);

            // calculates desirection movement velocity
            Vector3 targetVelocity = forward * (_speed * MoveSpeedMultiplier);
            if (!CanMove) targetVelocity = Vector3.zero;
            // adds velocity of surface under character, if character is stationary
            targetVelocity += SurfaceVelocity * (1f - Mathf.Abs(MoveInput.magnitude));
            // calculates acceleration required to reach desired velocity and applies air control if not grounded
            Vector3 velocityDiff = targetVelocity - Velocity;
            if (!_topDownMovement) velocityDiff.y = 0f;
            float control = IsGrounded ? 1f : _airControl;
            Vector3 acceleration = velocityDiff * (_acceleration * control);
            // zeros acceleration if airborne and not trying to move (allows for nice jumping arcs)
            if (!IsGrounded && !HasMoveInput) acceleration = Vector3.zero;
            // add gravity
            acceleration += GroundNormal * _gravity;

            _rigidbody.AddForce(acceleration);

            // rotates character towards movement direction
            if (_controlRotation && (IsGrounded || _airTurning))
            {
                transform.rotation = Quaternion.LookRotation(LookDirection);
            }
        }

        private void Update()
        {
            // check for the ground every frame
            IsGrounded = CheckGrounded();
        }

        private void LateUpdate()
        {
            // fix capsule collider rotation
            _collider.transform.rotation = Quaternion.identity;
        }

        private bool CheckGrounded()
        {
            // ignore ground checks if top-down
            if (_topDownMovement) return true;

            // configure layer mask for 2D raycast
            ContactFilter2D filter = new ContactFilter2D();
            filter.SetLayerMask(_groundMask);
            // raycast to find ground
            RaycastHit2D[] hits = new RaycastHit2D[4];
            RaycastHit2D groundHit = new RaycastHit2D();
            Physics2D.Raycast(_groundCheckStart, -transform.up, filter, hits, _groundCheckDistance);
            for (int i = 0; i < hits.Length; i++)
            {
                RaycastHit2D hit = hits[i];
                if(hit.collider != null && hit.collider != _collider)
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
            bool angleValid = Vector3.Angle(transform.up, groundHit.normal) < _maxSlopeAngle;
            if (angleValid)
            {
                // record last time character was grounded and set correct floor normal direction
                LastGroundedTime = Time.timeSinceLevelLoad;
                GroundNormal = groundHit.normal;
                LastGroundedPosition = transform.position;
                SurfaceObject = groundHit.collider.gameObject;
                return true;
            }

            SurfaceObject = null;
            return false;
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = IsGrounded ? Color.green : Color.red;
            Gizmos.DrawRay(_groundCheckStart, -transform.up * _groundCheckDistance);
        }
    }
}