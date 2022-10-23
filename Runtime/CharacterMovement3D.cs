﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
#pragma warning disable 649

// 3D implentation 
[RequireComponent(typeof(CapsuleCollider))]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(NavMeshAgent))]
public class CharacterMovement3D : CharacterMovement
{
    // public-read private-set properties
    public override Vector3 Velocity { get => _rigidbody.velocity; protected set => _rigidbody.velocity = value; }

    // properties
    public float TurnSpeedMultiplier { get; set; } = 1f;
    private Vector3 _groundCheckStart => transform.position + transform.up * _groundCheckOffset;

    // private fields
    private Rigidbody _rigidbody;
    private NavMeshAgent _navMeshAgent;
    
    private void Awake()
    {
        _rigidbody = GetComponent<Rigidbody>();
        _rigidbody.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        _rigidbody.interpolation = RigidbodyInterpolation.Interpolate;
        _rigidbody.useGravity = false;
        
        _navMeshAgent = GetComponent<NavMeshAgent>();
        _navMeshAgent.updatePosition = false;
        _navMeshAgent.updateRotation = false;
        
        LookDirection = transform.forward;
    }

    public override void SetMoveInput(Vector3 input)
    {
        if(!CanMove)
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

    // attempts a jump, will fail if not grounded
    public override void Jump()
    {
        if (!CanMove || !IsFudgeGrounded || LastGroundedDistance > _fudgeJumpMaxDistance) return;
        // calculate jump velocity from jump height and gravity
        float jumpVelocity = Mathf.Sqrt(2f * -_gravity * _jumpHeight);
        // override current y velocity but maintain x/z velocity
        Velocity = new Vector3(Velocity.x, jumpVelocity, Velocity.z);
    }

    // path to destination using navmesh
    public void MoveTo(Vector3 destination)
    {
        _navMeshAgent.SetDestination(destination);
    }

    // stop all movement
    public void Stop()
    {
        _navMeshAgent.ResetPath();
        SetMoveInput(Vector3.zero);
    }

    // allows character to move and rotate
    public void SetCanMove(bool canMove)
    {
        CanMove = canMove;
        if (!CanMove) Stop();
    }

    private void FixedUpdate()
    {
        // check for the ground
        IsGrounded = CheckGrounded();

        // overrides current input with pathing direction if MoveTo has been called
        if (_navMeshAgent.hasPath)
        {
            Vector3 nextPathPoint = _navMeshAgent.path.corners[1];
            Vector3 pathDir = (nextPathPoint - transform.position).normalized;
            SetMoveInput(pathDir);
            SetLookDirection(pathDir);
        }

        // syncs navmeshagent position with character position
        _navMeshAgent.Warp(transform.position);

        // find flattened movement vector based on ground normal
        Vector3 input = MoveInput;
        Vector3 right = Vector3.Cross(transform.up, input);
        Vector3 forward = Vector3.Cross(right, GroundNormal);

        // calculates desirection movement velocity
        Vector3 targetVelocity = forward * (_speed * MoveSpeedMultiplier);
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

        _rigidbody.AddForce(acceleration);

        // rotates character towards movement direction
        if (_controlRotation && HasTurnInput && (IsGrounded || _airTurning))
        {
            Quaternion targetRotation = Quaternion.LookRotation(LookDirection);
            Quaternion rotation = Quaternion.Slerp(transform.rotation, targetRotation, _turnSpeed * TurnSpeedMultiplier * Time.fixedDeltaTime);
            _rigidbody.MoveRotation(rotation);
        }
    }

    private bool CheckGrounded()
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