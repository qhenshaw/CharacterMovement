using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CharacterMovement
{
    [RequireComponent(typeof(CharacterMovementBase))]
    public class CharacterSpriteFlipper : MonoBehaviour
    {
        private CharacterMovementBase _movement;
        private SpriteRenderer _renderer;

        private void Start()
        {
            _movement = GetComponent<CharacterMovementBase>();
            _renderer = GetComponent<SpriteRenderer>();
        }

        private void Update()
        {
            if (!_movement.HasMoveInput) return;
            _renderer.flipX = _movement.MoveInput.x < 0f;
        }
    }
}