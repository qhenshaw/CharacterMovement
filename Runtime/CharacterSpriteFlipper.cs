using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CharacterMovement
{
    public class CharacterSpriteFlipper : MonoBehaviour
    {
        [field: SerializeField] private CharacterMovementBase Movement { get; set; }
        [field: SerializeField] private SpriteRenderer SpriteRenderer { get; set; }

        private void OnValidate()
        {
            if(Movement == null) Movement = GetComponent<CharacterMovementBase>();
            if(SpriteRenderer == null) SpriteRenderer = GetComponent<SpriteRenderer>();
        }

        private void Update()
        {
            if (!Movement.HasMoveInput) return;
            SpriteRenderer.flipX = Movement.MoveInput.x < 0f;
        }
    }
}