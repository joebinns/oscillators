using UnityEngine;
using Utilities;

namespace Oscillators
{
    /// <summary>
    /// A dampened oscillator using the objects transform local position.
    /// </summary>
    [DisallowMultipleComponent]
    public class Oscillator : MonoBehaviour
    {
        [Tooltip("The local position about which oscillations are centered.")]
        [SerializeField] private Vector3 _localEquilibriumPosition = Vector3.zero;
		[Tooltip("The local axes over which the oscillator applies force. Within range [0, 1].")]
		[SerializeField] private Vector3 _localForceScale = Vector3.one;
		[Tooltip("The greater the stiffness constant, the lesser the amplitude of oscillations.")]
        [SerializeField] private float _stiffness = 50f;
        [Tooltip("The greater the damper constant, the faster that oscillations will disappear.")]
        [SerializeField] private float _damper = 5f;
        [Tooltip("The greater the mass, the lesser the amplitude of oscillations. This value is ignored if a Rigidbody is present.")]
        [SerializeField] private float _mass = 1f; // TODO: Hide this variable if a Rigidybody component is present

        [Header("Limits")]
        [SerializeField] private bool _useLimits = false;
        [SerializeField] private Vector3 _minLocalEquilibriumPosition = -Vector3.one;
        [SerializeField] private Vector3 _maxLocalEquilibriumPosition = Vector3.one;

        public Vector3 LocalEquilibriumPosition {
            get => _localEquilibriumPosition;
            set => _localEquilibriumPosition = _useLimits ? MathsUtilities.ClampVector3(value, _minLocalEquilibriumPosition, _maxLocalEquilibriumPosition) : value;
        }

        private Rigidbody _rb;
        private Vector3 _previousDisplacement = Vector3.zero;
        private Vector3 _previousVelocity = Vector3.zero;

        /// <summary>
        /// Get the rigidbody component.
        /// </summary>
        private void Start() {
            _rb = GetComponent<Rigidbody>();
        }

        /// <summary>
        /// Update the position of the oscillator, by calculating and applying the restorative force.
        /// </summary>
        private void FixedUpdate() {
            Vector3 restoringForce = CalculateRestoringForce();
            ApplyForce(restoringForce);
        }

        /// <summary>
        /// Returns the damped restorative force of the oscillator.
        /// The magnitude of the restorative force is 0 at the equilibrium position and maximum at the amplitude of the oscillation.
        /// </summary>
        /// <returns>Damped restorative force of the oscillator.</returns>
        private Vector3 CalculateRestoringForce() {
            var parent = transform.parent;
            var position = transform.localPosition;
            var equilibrium = _localEquilibriumPosition;
            if (parent != null) {
                position = parent.TransformVector(position);
                equilibrium = parent.TransformVector(equilibrium);
            }
            Vector3 displacement = position - equilibrium; // Displacement from the rest point. Displacement is the difference in position.
            Vector3 deltaDisplacement = displacement - _previousDisplacement;
            _previousDisplacement = displacement;
            Vector3 velocity = deltaDisplacement / Time.fixedDeltaTime; // Kinematics. Velocity is the change-in-position over time.
            Vector3 force = HookesLaw(displacement, velocity);
            return (force);
        }

        /// <summary>
        /// Returns the damped Hooke's force for a given displacement and velocity.
        /// </summary>
        /// <param name="displacement">The displacement of the oscillator from the equilibrium position.</param>
        /// <param name="velocity">The local velocity of the oscillator.</param>
        /// <returns>Damped Hooke's force</returns>
        private Vector3 HookesLaw(Vector3 displacement, Vector3 velocity) {
            Vector3 force = (_stiffness * displacement) + (_damper * velocity); // Damped Hooke's law
            force = -force; // Take the negative of the force, since the force is restorative (attractive)
            return (force);
        }

        /// <summary>
        /// Adds a force to the oscillator. Updates the transform's local position.
        /// </summary>
        /// <param name="force">The force to be applied.</param>
        public void ApplyForce(Vector3 force) {
            var localForce = transform.InverseTransformVector(force);
            var scaledLocalForce = Vector3.Scale(localForce, _localForceScale);
            var scaledForce = transform.TransformVector(scaledLocalForce);

			if (_rb != null)
            {
                _rb.AddForce(scaledForce);
            }
            else
            {
                var displacement = CalculateDisplacementDueToForce(scaledForce);
                transform.localPosition += displacement;
            }
        }

        /// <summary>
        /// Returns the displacement that results from applying a force over a single fixed update.
        /// </summary>
        /// <param name="force">The causative force.</param>
        /// <returns>Displacement over a single fixed update.</returns>
        private Vector3 CalculateDisplacementDueToForce(Vector3 force) {
            Vector3 acceleration = force / _mass; // Newton's second law.
            Vector3 deltaVelocity = acceleration * Time.fixedDeltaTime; // Kinematics. Acceleration is the change in velocity over time.
            Vector3 velocity = deltaVelocity + _previousVelocity; // Calculating the updated velocity.
            _previousVelocity = velocity;
            Vector3 displacement = velocity * Time.fixedDeltaTime; // Kinematics. Velocity is the change-in-position over time.
            return (displacement);
        }
        
        /// <summary>
        /// Draws the oscillator bob (sphere) and the equilibrium (wire sphere).
        /// </summary>
        private void OnDrawGizmos() {
            if (!isActiveAndEnabled) return;
            
            Vector3 bob = transform.localPosition;
            Vector3 equilibrium = LocalEquilibriumPosition;
            Vector3 minEquilibrium = _minLocalEquilibriumPosition;
            Vector3 maxEquilibrium = _maxLocalEquilibriumPosition;
            var parent = transform.parent;
            if (parent != null)
            {
                bob = parent.TransformVector(bob);
                equilibrium = parent.TransformVector(equilibrium);
                minEquilibrium = parent.TransformVector(minEquilibrium);
                maxEquilibrium = parent.TransformVector(maxEquilibrium);
                bob += parent.position;
                equilibrium += parent.position;
                minEquilibrium += parent.position;
                maxEquilibrium += parent.position;
            }

            // Draw (wire) equilibrium position
            Color color = Color.green;
            Gizmos.color = color;
            Gizmos.DrawWireSphere(equilibrium, 0.1f);
            if (_useLimits) {
                color = Color.white;
                Gizmos.color = color;
                Gizmos.DrawWireSphere(minEquilibrium, 0.15f);
                Gizmos.DrawWireSphere(maxEquilibrium, 0.15f);
            }

            // Draw (solid) bob position
            // Color goes from green (0,1,0,0) to yellow (1,1,0,0) to red (1,0,0,0).
            float upperAmplitude = _stiffness * _mass / (3f * 100f); // Approximately the upper limit of the amplitude within regular use
            color.r = 2f * Mathf.Clamp(Vector3.Magnitude(bob - equilibrium) * upperAmplitude, 0f, 0.5f);
            color.g = 2f * (1f - Mathf.Clamp(Vector3.Magnitude(bob - equilibrium) * upperAmplitude, 0.5f, 1f));
            Gizmos.color = color;
            Gizmos.DrawSphere(bob, 0.125f);
            Gizmos.DrawLine(bob, equilibrium);
        }
    }
}
