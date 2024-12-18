using Unity.Cinemachine;
using System.Collections.Generic;
using UnityEngine;

namespace CharacterMovement
{
    public class CinemachineInputSensitivity : MonoBehaviour
    {
        [field: SerializeField] private float Multiplier { get; set; } = 1f;

        private List<InputAxisControllerBase<CinemachineInputAxisController.Reader>.Controller> _controllers;
        private float[] _originalGains;

        private void Start()
        {
            if(TryGetComponent(out CinemachineInputAxisController controllerComponent))
            {
                _controllers = controllerComponent.Controllers;
                _originalGains = new float[_controllers.Count];
                for (int i = 0; i < _controllers.Count; i++)
                {
                    _originalGains[i] = _controllers[i].Input.Gain;
                }

                SetMultiplierAll(Multiplier);
            }
        }

        public void SetMultiplierAll(float multiplier)
        {
            for (int i = 0; i < _controllers.Count; i++)
            {
                SetMultiplier(i, multiplier);
            }
        }

        public void SetMultiplier(int controllerIndex, float multiplier)
        {
            if(_controllers == null || _controllers.Count <= controllerIndex || _originalGains == null || _originalGains.Length <= controllerIndex)
            {
                Debug.LogError($"CinemachineInputAxisController axis missing at Index: {controllerIndex}");
            }
            _controllers[controllerIndex].Input.Gain = _originalGains[controllerIndex] * multiplier;
        }
    }
}