using Cinemachine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CharacterMovement
{
    public class CinemachineInputSensitivity : MonoBehaviour
    {
        private enum CameraInputMode
        {
            None,
            FreeLook,
            POV
        }

        [field: SerializeField] private float Multiplier { get; set; } = 1f;
        [field: SerializeField] private float TargetFPS { get; set; } = 60f;

        private Vector2 _sensitivity;
        private CinemachineFreeLook _freeLook;
        private CinemachinePOV _pov;
        private CameraInputMode _mode;

        private void Start()
        {
            if (TryGetComponent(out _freeLook))
            {
                _sensitivity.x = _freeLook.m_XAxis.m_MaxSpeed;
                _sensitivity.y = _freeLook.m_YAxis.m_MaxSpeed;
                _mode = CameraInputMode.FreeLook;
            }
            else if (TryGetComponent(out CinemachineVirtualCamera vcam))
            {
                _pov = vcam.GetCinemachineComponent<CinemachinePOV>();
                if (_pov != null)
                {
                    _sensitivity.x = _pov.m_HorizontalAxis.m_MaxSpeed;
                    _sensitivity.y = _pov.m_VerticalAxis.m_MaxSpeed;
                    _mode = CameraInputMode.POV;
                }
            }
        }

        private void Update()
        {
            float deltaTime = Mathf.Clamp(Time.smoothDeltaTime, 0.001f, 1f);
            float currentFPS = 1f / deltaTime;
            Vector2 adjustedSensitivity = _sensitivity * currentFPS / TargetFPS * Multiplier;

            switch (_mode)
            {
                case CameraInputMode.None:
                    break;
                case CameraInputMode.FreeLook:
                    _freeLook.m_XAxis.m_MaxSpeed = adjustedSensitivity.x;
                    _freeLook.m_YAxis.m_MaxSpeed = adjustedSensitivity.y;
                    break;
                case CameraInputMode.POV:
                    _pov.m_HorizontalAxis.m_MaxSpeed = adjustedSensitivity.x;
                    _pov.m_VerticalAxis.m_MaxSpeed = adjustedSensitivity.y;
                    break;
            }
        }
    }
}