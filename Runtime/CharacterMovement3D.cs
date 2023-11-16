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
        [Header("Avoidance")]
        [SerializeField] protected bool _enableAvoidance = false;
        [SerializeField, Range(0f, 1f)] protected float _speedVariation = 0.5f;
        [SerializeField] protected float _neighborDistance = 3f;
        [SerializeField] protected float _cornerNeighborDistance = 1f;
        [SerializeField] protected LayerMask _neighborMask;
        [SerializeField] protected int _maxNeighbors = 8;
        [SerializeField] protected bool _clampToNavMesh = true;
        [SerializeField] private float _clampLookAheadTime = 0.25f;
        [SerializeField] private float _clampSearchRadius = 1f;
        protected float _variationNoiseOffset;
        protected Collider[] _neighborHits;

        // public-read private-set properties
        public override Vector3 Velocity { get => _rigidbody.velocity; protected set => _rigidbody.velocity = value; }

        // properties
        public float TurnSpeedMultiplier { get; set; } = 1f;
        protected Vector3 _groundCheckStart => transform.position + transform.up * _groundCheckOffset;

        // private fields
        [Header("Components")]
        [SerializeField] protected Rigidbody _rigidbody;
        [SerializeField] protected NavMeshAgent _navMeshAgent;
        [SerializeField] protected CapsuleCollider _capsuleCollider;

        protected virtual void OnValidate()
        {
            if(_rigidbody == null) _rigidbody = GetComponent<Rigidbody>();
            if(_navMeshAgent == null) _navMeshAgent = GetComponent<NavMeshAgent>();
            if(_capsuleCollider == null) _capsuleCollider = GetComponent<CapsuleCollider>();

            _rigidbody.freezeRotation = true;
            _rigidbody.useGravity = false;
            _rigidbody.interpolation = RigidbodyInterpolation.Interpolate;

            _navMeshAgent = GetComponent<NavMeshAgent>();
            _navMeshAgent.height = _height;
            _navMeshAgent.radius = _radius;

            // adjust capsule height/radius based on fields
            _capsuleCollider.height = _height;
            _capsuleCollider.center = new Vector3(0f, _height * 0.5f, 0f);
            _capsuleCollider.radius = _radius;
        }

        protected virtual void Awake()
        {
            // assign frictionless physic material
            _capsuleCollider.material = new PhysicMaterial("NoFriction") { staticFriction = 0f, dynamicFriction = 0f, frictionCombine = PhysicMaterialCombine.Minimum };

            // disable NavMeshAgent movement
            _navMeshAgent.updatePosition = false;
            _navMeshAgent.updateRotation = false;

            // match look direction to current facing
            LookDirection = transform.forward;

            // set up avoidance values
            _neighborHits = new Collider[_maxNeighbors];
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
            if (!CanMove || direction.magnitude < 0.1f)
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
            float jumpVelocity = Mathf.Sqrt(2f * -_gravity * _jumpHeight);
            // override current y velocity but maintain x/z velocity
            Velocity = new Vector3(Velocity.x, jumpVelocity, Velocity.z);
        }

        // path to destination using navmesh
        public virtual void MoveTo(Vector3 destination)
        {
            _navMeshAgent.SetDestination(destination);
        }

        // stop all movement
        public virtual void Stop()
        {
            _navMeshAgent.ResetPath();
            SetMoveInput(Vector3.zero);
        }

        protected virtual void FixedUpdate()
        {
            // check for the ground
            IsGrounded = CheckGrounded();

            // overrides current input with pathing direction if MoveTo has been called
            if (_navMeshAgent.hasPath)
            {
                Vector3 nextPathPoint = _navMeshAgent.path.corners[1];
                Vector3 pathDir = (nextPathPoint - transform.position).normalized;
                // override direction if avoidance is enabled
                if(_enableAvoidance)
                {
                    float neighborDistance = _neighborDistance;
                    if (_navMeshAgent.path.corners.Length > 2) neighborDistance = _cornerNeighborDistance;
                    pathDir = GetAvoidanceDirection(nextPathPoint, neighborDistance);

                    if (_clampToNavMesh)
                    {
                        Vector3 pathPoint = transform.position + pathDir * _speed * _clampLookAheadTime;
                        Vector3 clampedPathPoint = ClampToNavMesh(pathPoint, _clampSearchRadius);
                        Debug.DrawLine(transform.position, pathPoint, Color.magenta);
                        pathDir = (clampedPathPoint - transform.position).normalized;
                    }
                }
                SetMoveInput(pathDir);
                if(_lookInMoveDirection) SetLookDirection(pathDir);

                // stop off destination reached
                if (_stoppingDistance > 0f && Vector3.Distance(_navMeshAgent.destination, transform.position) < _stoppingDistance)
                {
                    Stop();
                }
            }

            // syncs navmeshagent position with character position
            _navMeshAgent.nextPosition = transform.position;

            // find flattened movement vector based on ground normal
            Vector3 input = MoveInput;
            Vector3 right = Vector3.Cross(transform.up, input);
            Vector3 forward = Vector3.Cross(right, GroundNormal);

            // vary character speed when using avoidance
            float speed = _speed;
            if(_enableAvoidance)
            {
                float noise = Mathf.PerlinNoise(Time.time, _variationNoiseOffset) * 2f - 1f;
                speed = _speed * (1f + noise * _speedVariation);
            }

            // calculates desirection movement velocity
            Vector3 targetVelocity = forward * (speed * MoveSpeedMultiplier);
            if (!CanMove) targetVelocity = Vector3.zero;
            // adds velocity of surface under character, if character is stationary
            targetVelocity += SurfaceVelocity * (1f - Mathf.Abs(MoveInput.magnitude));
            // calculates acceleration required to reach desired velocity and applies air control if not grounded
            Vector3 velocityDiff = targetVelocity - Velocity;
            velocityDiff.y = 0f;
            float control = IsGrounded ? 1f : _airControl;
            Vector3 acceleration = velocityDiff * (_acceleration * control);
            // zeros acceleration if airborne and not trying to move (allows for nice jumping arcs)
            if (!IsGrounded && !HasMoveInput) acceleration = Vector3.zero;
            // add gravity
            acceleration += GroundNormal * _gravity;

            _rigidbody.AddForce(acceleration * _rigidbody.mass);
        }

        protected virtual void Update()
        {
            // rotates character towards movement direction
            if (_controlRotation && HasTurnInput && (IsGrounded || _airTurning))
            {
                Quaternion targetRotation = Quaternion.LookRotation(LookDirection);
                Quaternion rotation = Quaternion.Slerp(transform.rotation, targetRotation, _turnSpeed * TurnSpeedMultiplier * Time.deltaTime);
                _rigidbody.MoveRotation(rotation);
            }
        }

        protected virtual bool CheckGrounded()
        {
            // raycast to find ground
            bool hit = Physics.Raycast(_groundCheckStart, -transform.up, out RaycastHit hitInfo, _groundCheckDistance, _groundMask);

            // set default ground surface normal and SurfaceVelocity
            GroundNormal = Vector3.up;
            SurfaceVelocity = Vector3.zero;

            // if ground wasn't hit, character is not grounded
            if (!hit) return false;

            // gets velocity of surface underneath character if applicable
            if (hitInfo.rigidbody != null) SurfaceVelocity = hitInfo.rigidbody.velocity;

            // test angle between character up and ground, angles above _maxSlopeAngle are invalid
            bool angleValid = Vector3.Angle(transform.up, hitInfo.normal) < _maxSlopeAngle;
            if (angleValid)
            {
                // record last time character was grounded and set correct floor normal direction
                LastGroundedTime = Time.timeSinceLevelLoad;
                GroundNormal = hitInfo.normal;
                LastGroundedPosition = transform.position;
                SurfaceObject = hitInfo.collider.gameObject;
                if(_parentToSurface) transform.SetParent(SurfaceObject.transform);
                return true;
            }

            SurfaceObject = null;
            if(_parentToSurface) transform.SetParent(null);
            return false;
        }

        // gets move direction adjusted to avoid neighbors
        protected Vector3 GetAvoidanceDirection(Vector3 destination, float neighborDistance)
        {
            Vector3 position = transform.position;

            Vector3 separation = Vector3.zero;
            Vector3 alignment = transform.forward;
            Vector3 cohesion = destination;

            int hitCount = Physics.OverlapSphereNonAlloc(position, neighborDistance, _neighborHits, _neighborMask);
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
            Gizmos.DrawRay(_groundCheckStart, -transform.up * _groundCheckDistance);

            if(_enableAvoidance)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(transform.position, _neighborDistance);
            }
        }
    }
}