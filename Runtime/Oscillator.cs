using UnityEngine;
using Utilities;

namespace Oscillators
{
	/// <summary>
	/// A dampened oscillator using the objects transform local position.
	/// </summary>
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

		private Rigidbody _rigidbody;
		private Vector3 _previousDisplacement = Vector3.zero;
		private Vector3 _previousVelocity = Vector3.zero;

		private void Awake()
		{
			_rigidbody = GetComponent<Rigidbody>();
		}

		private void Update()
		{
			if (_rigidbody != null) continue;

			var restoringForce = CalculateRestoringForce(Time.deltaTime);
			ApplyForce(restoringForce, Time.deltaTime);
		}

		private void FixedUpdate()
		{
			if (_rigidbody == null) continue;

			var restoringForce = CalculateRestoringForce(Time.fixedDeltaTime);
			ApplyForce(restoringForce, Time.fixedDeltaTime);
		}

		private Vector3 CalculateRestoringForce(var deltaTime)
		{
			var parent = transform.parent;
			var position = transform.localPosition;
			var equilibrium = _localEquilibriumPosition;
			if (parent != null)
			{
				position = parent.TransformVector(position);
				equilibrium = parent.TransformVector(equilibrium);
			}
			var displacement = position - equilibrium;
			var deltaDisplacement = displacement - _previousDisplacement;
			_previousDisplacement = displacement;
			var velocity = deltaDisplacement / deltaTime;
			var force = HookesLaw(displacement, velocity);
			return (force);
		}

		private Vector3 HookesLaw(Vector3 displacement, Vector3 velocity)
		{
			var force = (_stiffness * displacement) + (_damper * velocity);
			force = -force;
			return (force);
		}

		public void ApplyForce(Vector3 force, float deltaTime)
		{
			var localForce = transform.InverseTransformVector(force);
			var scaledLocalForce = Vector3.Scale(localForce, _localForceScale);
			var scaledForce = transform.TransformVector(scaledLocalForce);

			if (_rigidbody != null)
			{
				_rigidbody.AddForce(scaledForce);
			}
			else
			{
				var displacement = CalculateDisplacementDueToForce(scaledForce, deltaTime);
				transform.localPosition += displacement;
			}
		}

		private Vector3 CalculateDisplacementDueToForce(Vector3 force, float deltaTime)
		{
			var acceleration = force / _mass;
			var deltaVelocity = acceleration * deltaTime;
			var velocity = _previousVelocity + 0.5f * deltaVelocity;
			var displacement = velocity * deltaTime;
			velocity += 0.5f * deltaVelocity;
			_previousVelocity = velocity;
			return (displacement);
		}

		private void OnDrawGizmos()
		{
			if (!isActiveAndEnabled) return;

			var bob = transform.localPosition;
			var equilibrium = LocalEquilibriumPosition;
			var minEquilibrium = _minLocalEquilibriumPosition;
			var maxEquilibrium = _maxLocalEquilibriumPosition;
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
			Gizmos.DrawWireSphere(equilibrium, 0.1f)

			// Draw (solid) bob position
			// Color goes from green (0,1,0,0) to yellow (1,1,0,0) to red (1,0,0,0).
			var upperAmplitude = _stiffness * _mass / (3f * 100f); // Approximately the upper limit of the amplitude within regular use
			color.r = 2f * Mathf.Clamp(Vector3.Magnitude(bob - equilibrium) * upperAmplitude, 0f, 0.5f);
			color.g = 2f * (1f - Mathf.Clamp(Vector3.Magnitude(bob - equilibrium) * upperAmplitude, 0.5f, 1f));
			Gizmos.color = color;
			Gizmos.DrawSphere(bob, 0.125f);
			Gizmos.DrawLine(bob, equilibrium);
		}
	}
}
