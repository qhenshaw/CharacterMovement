using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Random = UnityEngine.Random;

namespace CharacterMovement
{
    // 3D implentation 
    [RequireComponent(typeof(CapsuleCollider))]
    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(NavMeshAgent))]
    public class CharacterMovement3D : CharacterMovementBase
    {
        // all avoidance fields
        [field: Header("Avoidance")]
        [field: SerializeField] protected bool EnableAvoidance { get; set; } = false;
        [field: SerializeField, Range(0f, 1f)] protected float SpeedVariation { get; set; } = 0.5f;
        [field: SerializeField] protected float NeighborDistance { get; set; } = 3f;
        [field: SerializeField] protected float CornerNeighborDistance { get; set; } = 1f;
        [field: SerializeField] protected LayerMask NeighborMask { get; set; }
        [field: SerializeField] protected int MaxNeighbors { get; set; } = 8;
        [field: SerializeField] protected bool IsClampedToNavMesh { get; set; } = true;
        [field: SerializeField] protected float ClampLookAheadTime { get; set; } = 0.25f;
        [field: SerializeField] protected float ClampSearchRadius { get; set; } = 1f;
        protected float _variationNoiseOffset;
        protected Collider[] _neighborHits;

        [field: Header("Components")]
        [field: SerializeField] protected Rigidbody Rigidbody { get; set; }
        [field: SerializeField] protected NavMeshAgent NavMeshAgent { get; set; }
        [field: SerializeField] protected CapsuleCollider CapsuleCollider { get; set; }

        // useful properties
        public override Vector3 Velocity { get => Rigidbody.velocity; protected set => Rigidbody.velocity = value; }
        public float TurnSpeedMultiplier { get; set; } = 1f;
        protected Vector3 GroundCheckStart => transform.position + transform.up * GroundCheckOffset;

        protected virtual void OnValidate()
        {
            if(Rigidbody == null) Rigidbody = GetComponent<Rigidbody>();
            if(NavMeshAgent == null) NavMeshAgent = GetComponent<NavMeshAgent>();
            if(CapsuleCollider == null) CapsuleCollider = GetComponent<CapsuleCollider>();

            Rigidbody.freezeRotation = true;
            Rigidbody.useGravity = false;
            Rigidbody.interpolation = RigidbodyInterpolation.Interpolate;

            NavMeshAgent = GetComponent<NavMeshAgent>();
            NavMeshAgent.height = Height;
            NavMeshAgent.radius = Radius;

            // adjust capsule height/radius based on fields
            CapsuleCollider.height = Height;
            CapsuleCollider.center = new Vector3(0f, Height * 0.5f, 0f);
            CapsuleCollider.radius = Radius;
        }

        protected virtual void Awake()
        {
            // assign frictionless physic material
            CapsuleCollider.material = new PhysicMaterial("NoFriction") { staticFriction = 0f, dynamicFriction = 0f, frictionCombine = PhysicMaterialCombine.Minimum };

            // disable NavMeshAgent movement
            NavMeshAgent.updatePosition = false;
            NavMeshAgent.updateRotation = false;

            // match look direction to current facing
            LookDirection = transform.forward;

            // set up avoidance values
            _neighborHits = new Collider[MaxNeighbors];
            _variationNoiseOffset = Random.value * 10f;
        }

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
            // remove y component of movement but retain overall magnitude
            Vector3 flattened = new Vector3(input.x, 0f, input.z);
            flattened = flattened.normalized * input.magnitude;
            MoveInput = flattened;
            // finds movement input as local direction rather than world direction
            LocalMoveInput = transform.InverseTransformDirection(MoveInput);
        }

        // sets character look direction, flattening y-value
        public override void SetLookDirection(Vector3 direction)
        {
            if (!CanTurn || direction.magnitude < 0.1f)
            {
                HasTurnInput = false;
                return;
            }
            HasTurnInput = true;
            LookDirection = new Vector3(direction.x, 0f, direction.z).normalized;
        }

        public override void SetLookPosition(Vector3 position)
        {
            Vector3 direction = Vector3.ClampMagnitude(position - transform.position, 1f);
            SetLookDirection(direction);
        }

        // attempts a jump, will fail if not grounded
        public override void Jump()
        {
            if (!CanMove || !CanCoyoteJump) return;
            // calculate jump velocity from jump height and gravity
            float jumpVelocity = Mathf.Sqrt(2f * -Gravity * JumpHeight);
            // override current y velocity but maintain x/z velocity
            Velocity = new Vector3(Velocity.x, jumpVelocity, Velocity.z);
        }

        // path to destination using navmesh
        public virtual void MoveTo(Vector3 destination)
        {
            if (!NavMeshAgent.isActiveAndEnabled || !NavMeshAgent.isOnNavMesh) return;
            NavMeshAgent.SetDestination(destination);
        }

        // stop all movement
        public virtual void Stop()
        {
            SetMoveInput(Vector3.zero);
            if (!NavMeshAgent.isActiveAndEnabled || !NavMeshAgent.isOnNavMesh) return;
            NavMeshAgent.ResetPath();
        }

        protected virtual void FixedUpdate()
        {
            // check for the ground
            IsGrounded = CheckGrounded();

            // overrides current input with pathing direction if MoveTo has been called
            if (NavMeshAgent.hasPath)
            {
                Vector3 nextPathPoint = NavMeshAgent.path.corners[1];
                Vector3 pathDir = (nextPathPoint - transform.position).normalized;
                // override direction if avoidance is enabled
                if(EnableAvoidance)
                {
                    float neighborDistance = NeighborDistance;
                    if (NavMeshAgent.path.corners.Length > 2) neighborDistance = CornerNeighborDistance;
                    pathDir = GetAvoidanceDirection(nextPathPoint, neighborDistance);

                    if (IsClampedToNavMesh)
                    {
                        Vector3 pathPoint = transform.position + pathDir * Speed * ClampLookAheadTime;
                        Vector3 clampedPathPoint = ClampToNavMesh(pathPoint, ClampSearchRadius);
                        Debug.DrawLine(transform.position, pathPoint, Color.magenta);
                        pathDir = (clampedPathPoint - transform.position).normalized;
                    }
                }
                SetMoveInput(pathDir);
                if(LookInMoveDirection) SetLookDirection(pathDir);

                // stop off destination reached
                if (StoppingDistance > 0f && Vector3.Distance(NavMeshAgent.destination, transform.position) < StoppingDistance)
                {
                    Stop();
                }
            }

            // syncs navmeshagent position with character position
            NavMeshAgent.nextPosition = transform.position;

            // find flattened movement vector based on ground normal
            Vector3 input = MoveInput;
            Vector3 right = Vector3.Cross(transform.up, input);
            Vector3 forward = Vector3.Cross(right, GroundNormal);

            // vary character speed when using avoidance
            float speed = Speed;
            if(EnableAvoidance)
            {
                float noise = Mathf.PerlinNoise(Time.time, _variationNoiseOffset) * 2f - 1f;
                speed = Speed * (1f + noise * SpeedVariation);
            }

            // calculates desirection movement velocity
            Vector3 targetVelocity = forward * (speed * MoveSpeedMultiplier);
            if (!CanMove) targetVelocity = Vector3.zero;
            // adds velocity of surface under character, if character is stationary
            targetVelocity += SurfaceVelocity * (1f - Mathf.Abs(MoveInput.magnitude));
            // calculates acceleration required to reach desired velocity and applies air control if not grounded
            Vector3 velocityDiff = targetVelocity - Velocity;
            velocityDiff.y = 0f;
            float control = IsGrounded ? 1f : AirControl;
            Vector3 acceleration = velocityDiff * (Acceleration * control);
            // zeros acceleration if airborne and not trying to move (allows for nice jumping arcs)
            if (!IsGrounded && !HasMoveInput) acceleration = Vector3.zero;
            // add gravity
            acceleration += GroundNormal * Gravity;

            Rigidbody.AddForce(acceleration * Rigidbody.mass);
        }

        protected virtual void Update()
        {
            // rotates character towards movement direction
            if (ControlRotation && HasTurnInput && (IsGrounded || AirTurning))
            {
                Quaternion rotation = Rigidbody.rotation;
                if(!Fix3DSpriteRotation)
                {
                    Quaternion targetRotation = Quaternion.LookRotation(LookDirection);
                    rotation = Quaternion.Slerp(transform.rotation, targetRotation, TurnSpeed * TurnSpeedMultiplier * Time.deltaTime);
                }   // rotate sprite character properly
                else if (Fix3DSpriteRotation && Mathf.Abs(MoveInput.x) > 0.2f)
                {
                    float spriteAngle = LookDirection.x > 0 ? 0f : 180f;
                    rotation = Quaternion.Euler(0f, spriteAngle, 0f);
                }
                Rigidbody.MoveRotation(rotation);
            }
        }

        protected virtual bool CheckGrounded()
        {
            // raycast to find ground
            bool hit = Physics.Raycast(GroundCheckStart, -transform.up, out RaycastHit hitInfo, GroundCheckDistance, GroundMask);

            // set default ground surface normal and SurfaceVelocity
            GroundNormal = Vector3.up;
            SurfaceVelocity = Vector3.zero;

            // if ground wasn't hit, character is not grounded
            if (!hit) return false;

            // gets velocity of surface underneath character if applicable
            if (hitInfo.rigidbody != null) SurfaceVelocity = hitInfo.rigidbody.velocity;

            // test angle between character up and ground, angles above _maxSlopeAngle are invalid
            bool angleValid = Vector3.Angle(transform.up, hitInfo.normal) < MaxSlopeAngle;
            if (angleValid)
            {
                // record last time character was grounded and set correct floor normal direction
                LastGroundedTime = Time.timeSinceLevelLoad;
                GroundNormal = hitInfo.normal;
                LastGroundedPosition = transform.position;
                SurfaceObject = hitInfo.collider.gameObject;
                if(ParentToSurface) transform.SetParent(SurfaceObject.transform);
                return true;
            }

            SurfaceObject = null;
            if(ParentToSurface) transform.SetParent(null);
            return false;
        }

        // gets move direction adjusted to avoid neighbors
        protected Vector3 GetAvoidanceDirection(Vector3 destination, float neighborDistance)
        {
            Vector3 position = transform.position;

            Vector3 separation = Vector3.zero;
            Vector3 alignment = transform.forward;
            Vector3 cohesion = destination;

            int hitCount = Physics.OverlapSphereNonAlloc(position, neighborDistance, _neighborHits, NeighborMask);
            int neighborCount = 0;
            for (int i = 0; i < hitCount; i++)
            {
                GameObject neighbor = _neighborHits[i].gameObject;
                if (neighbor == gameObject) continue;
                neighborCount++;
                separation += GetSeparationVector(neighbor.transform, neighborDistance);
                alignment += neighbor.transform.forward;
                cohesion += neighbor.transform.position;
            }

            float average = 1f / (neighborCount + 1);
            alignment *= average;
            cohesion *= average;
            cohesion = (cohesion - position).normalized;

            Vector3 direction = separation + alignment + cohesion;
            return Vector3.ClampMagnitude(direction, 1f);
        }

        // calculates separation strength/direction from neigbor
        private Vector3 GetSeparationVector(Transform target, float neighborDistance)
        {
            Vector3 diff = transform.position - target.transform.position;
            float diffLen = diff.magnitude;
            float scaler = Mathf.Clamp01(1.0f - diffLen / neighborDistance);
            return diff * (scaler / diffLen);
        }

        protected Vector3 ClampToNavMesh(Vector3 position, float searchRadius)
        {
            if(NavMesh.SamplePosition(position, out NavMeshHit hit, searchRadius, NavMesh.AllAreas))
            {
                return hit.position;
            }

            return position;
        }

        // check for landing on the ground
        protected virtual void OnCollisionEnter(Collision collision)
        {
            float landingCollisionMaxDistance = 0.25f;
            Vector3 point = collision.contacts[0].point;
            if(Vector3.Distance(point, transform.position) < landingCollisionMaxDistance)
            {
                OnGrounded.Invoke(collision.gameObject);
            }
        }

        protected virtual void OnDrawGizmosSelected()
        {
            Gizmos.color = IsGrounded ? Color.green : Color.red;
            Gizmos.DrawRay(GroundCheckStart, -transform.up * GroundCheckDistance);

            if(EnableAvoidance)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(transform.position, NeighborDistance);
            }
        }
    }
}