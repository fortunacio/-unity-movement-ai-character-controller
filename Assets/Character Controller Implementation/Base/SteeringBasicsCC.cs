using UnityEngine;
using System.Collections.Generic;


[RequireComponent(typeof(CharacterLocomotion))]
public class SteeringBasicsCC : MonoBehaviour
{
    [Header("General")]

    public float maxVelocity = 3.5f;

    public float maxAcceleration = 10f;

    public float turnSpeed = 20f;

    [Header("Arrive")]

    /// <summary>
    /// The radius from the target that means we are close enough and have arrived
    /// </summary>
    public float targetRadius = 0.005f;

    /// <summary>
    /// The radius from the target where we start to slow down
    /// </summary>
    public float slowRadius = 1f;

    /// <summary>
    /// The time in which we want to achieve the targetSpeed
    /// </summary>
    public float timeToTarget = 0.1f;


    [Header("Look Direction Smoothing")]

    /// <summary>
    /// Smoothing controls if the character's look direction should be an
    /// average of its previous directions (to smooth out momentary changes
    /// in directions)
    /// </summary>
    public bool smoothing = true;
    public int numSamplesForSmoothing = 5;
    Queue<Vector3> velocitySamples = new Queue<Vector3>();

    [Header("Separation")]
    /// <summary>
    /// The maximum acceleration for separation
    /// </summary>
    public float sepMaxAcceleration = 25;

    /// <summary>
    /// This should be the maximum separation distance possible between a
    /// separation target and the character. So it should be: separation
    /// sensor radius + max target radius
    /// </summary>
    public float maxSepDist = 1f;

    [Header("Pursuit")]
    /// <summary>
    /// Maximum prediction time the pursue will predict in the future
    /// </summary>
    public float maxPrediction = 1f;

    [Header("Wander")]

    public float wanderRadius = 1.2f;
    public float wanderDistance = 2f;
    /// <summary>
    /// Maximum amount of random displacement a second
    /// </summary>
    public float wanderJitter = 40f;
    Vector3 wanderTarget;

    [Header("Collision Avoidance")]
    public float maxAccelerationOnAvoid = 15f;

    [Tooltip("How much space can be between two characters before they are considered colliding")]
    public float distanceBetween;


    CharacterLocomotion locomotion;

    void Awake()
    {
        locomotion = GetComponent<CharacterLocomotion>();

        SetupWanderTarget();
    }

    void SetupWanderTarget() {
        /* Set up the wander target. Doing this in Start() because the MovementAIRigidbody
         * sets itself up in Awake(). */
        float theta = Random.value * 2 * Mathf.PI;

        /* Create a vector to a target position on the wander circle */

        wanderTarget = new Vector3(wanderRadius * Mathf.Cos(theta), 0f, wanderRadius * Mathf.Sin(theta));
    }

    /// <summary>
    /// Updates the velocity of the current game object by the given linear
    /// acceleration
    /// </summary>
    public void Steer(Vector3 linearAcceleration)
    {
        locomotion.Velocity += linearAcceleration * Time.fixedDeltaTime;
        //var velocity = pm.Velocity + linearAcceleration * Time.fixedDeltaTime;
        //pm.Move(velocity);

        if (locomotion.Velocity.magnitude > maxVelocity)
        {
            //pm.Move(pm.Velocity.normalized * maxVelocity);
            locomotion.Velocity = locomotion.Velocity.normalized * maxVelocity;
        }
    }


    public Vector3 Seek(Vector3 targetPosition)
    {
        return Seek(targetPosition, maxAcceleration);
    }

    /// <summary>
    /// A seek steering behavior. Will return the steering for the current game object to seek a given position
    /// </summary>
    public Vector3 Seek(Vector3 targetPosition, float maxSeekAccel)
    {
        /* Get the direction */
        Vector3 acceleration = targetPosition - transform.position;
        acceleration.y = 0;
        acceleration.Normalize();

        /* Accelerate to the target */
        acceleration *= maxSeekAccel;

        return acceleration;
    }

    public Vector3 Separation(ICollection<CharacterLocomotion> targets)
    {
        Vector3 acceleration = Vector3.zero;

        foreach (CharacterLocomotion r in targets)
        {
            /* Get the direction and distance from the target */
            Vector3 direction = locomotion.ColliderPosition - r.ColliderPosition;
            float dist = direction.magnitude;

            if (dist < maxSepDist)
            {
                /* Calculate the separation strength (can be changed to use inverse square law rather than linear) */
                var strength = sepMaxAcceleration * (maxSepDist - dist) / (maxSepDist - locomotion.Radius - r.Radius);

                /* Added separation acceleration to the existing steering */
                direction.Normalize();
                acceleration += direction * strength;
            }
        }

        return acceleration;
    }

    public Vector3 OffsetPursuit(CharacterLocomotion target, Vector3 offset)
    {
        Vector3 targetPos;
        return OffsetPursuit(target, offset, out targetPos);
    }

    public Vector3 OffsetPursuit(CharacterLocomotion target, Vector3 offset, out Vector3 targetPos)
    {
        Vector3 worldOffsetPos = target.Position + target.transform.TransformDirection(offset);

        /* Calculate the distance to the offset point */
        Vector3 displacement = worldOffsetPos - transform.position;
        float distance = displacement.magnitude;

        /* Get the character's speed */
        float speed = locomotion.Velocity.magnitude;

        /* Calculate the prediction time */
        float prediction;
        if (speed <= distance / maxPrediction)
        {
            prediction = maxPrediction;
        }
        else
        {
            prediction = distance / speed;
        }

        /* Put the target together based on where we think the target will be */
        targetPos = worldOffsetPos + target.Velocity * prediction;

        return Arrive(targetPos);
    }



    public Vector3 Pursure(CharacterLocomotion target)
    {
        /* Calculate the distance to the target */
        Vector3 displacement = target.Position - transform.position;
        float distance = displacement.magnitude;

        /* Get the character's speed */
        float speed = locomotion.Velocity.magnitude;

        /* Calculate the prediction time */
        float prediction;
        if (speed <= distance / maxPrediction)
        {
            prediction = maxPrediction;
        }
        else
        {
            prediction = distance / speed;
        }

        /* Put the target together based on where we think the target will be */
        Vector3 explicitTarget = target.Position + target.Velocity * prediction;

        //Debug.DrawLine(transform.position, explicitTarget);

        return Seek(explicitTarget);
    }

    public Vector3 Wander() {
        /* Get the jitter for this time frame */
        float jitter = wanderJitter * Time.deltaTime;

        /* Add a small random vector to the target's position */
        wanderTarget += new Vector3(Random.Range(-1f, 1f) * jitter, 0f, Random.Range(-1f, 1f) * jitter);

        /* Make the wanderTarget fit on the wander circle again */
        wanderTarget.Normalize();
        wanderTarget *= wanderRadius;

        /* Move the target in front of the character */
        Vector3 targetPosition = transform.position + transform.right * wanderDistance + wanderTarget;

        //Debug.DrawLine(transform.position, targetPosition);

        return Seek(targetPosition);
    }

    public Vector3 CollisionAvoidance(ICollection<CharacterLocomotion> targets)
    {
        Vector3 acceleration = Vector3.zero;

        /* 1. Find the target that the character will collide with first */

        /* The first collision time */
        float shortestTime = float.PositiveInfinity;

        /* The first target that will collide and other data that
         * we will need and can avoid recalculating */
        CharacterLocomotion firstTarget = null;
        float firstMinSeparation = 0, firstDistance = 0, firstRadius = 0;
        Vector3 firstRelativePos = Vector3.zero, firstRelativeVel = Vector3.zero;

        foreach (CharacterLocomotion r in targets)
        {
            /* Calculate the time to collision */
            Vector3 relativePos = locomotion.ColliderPosition - r.ColliderPosition;
            Vector3 relativeVel = locomotion.Velocity - r.Velocity;
            float distance = relativePos.magnitude;
            float relativeSpeed = relativeVel.magnitude;

            if (relativeSpeed == 0)
            {
                continue;
            }

            float timeToCollision = -1 * Vector3.Dot(relativePos, relativeVel) / (relativeSpeed * relativeSpeed);

            /* Check if they will collide at all */
            Vector3 separation = relativePos + relativeVel * timeToCollision;
            float minSeparation = separation.magnitude;

            if (minSeparation > locomotion.Radius + r.Radius + distanceBetween)
            {
                continue;
            }

            /* Check if its the shortest */
            if (timeToCollision > 0 && timeToCollision < shortestTime)
            {
                shortestTime = timeToCollision;
                firstTarget = r;
                firstMinSeparation = minSeparation;
                firstDistance = distance;
                firstRelativePos = relativePos;
                firstRelativeVel = relativeVel;
                firstRadius = r.Radius;
            }
        }

        /* 2. Calculate the steering */

        /* If we have no target then exit */
        if (firstTarget == null)
        {
            return acceleration;
        }

        /* If we are going to collide with no separation or if we are already colliding then 
         * steer based on current position */
        if (firstMinSeparation <= 0 || firstDistance < locomotion.Radius + firstRadius + distanceBetween)
        {
            acceleration = locomotion.ColliderPosition - firstTarget.ColliderPosition;
        }
        /* Else calculate the future relative position */
        else
        {
            acceleration = firstRelativePos + firstRelativeVel * shortestTime;
        }

        /* Avoid the target */
        acceleration.y = 0;
        acceleration.Normalize();
        acceleration *= maxAcceleration;

        return acceleration;
    }

    /// <summary>
    /// Makes the current game object look where he is going
    /// </summary>
    public void LookWhereYoureGoing()
    {
        Vector3 direction = locomotion.Velocity;

        if (smoothing)
        {
            if (velocitySamples.Count == numSamplesForSmoothing)
            {
                velocitySamples.Dequeue();
            }

            velocitySamples.Enqueue(locomotion.Velocity);

            direction = Vector3.zero;

            foreach (Vector3 v in velocitySamples)
            {
                direction += v;
            }

            direction /= velocitySamples.Count;
        }

        LookAtDirection(direction);
    }

    public void LookAtDirection(Vector3 direction)
    {
        direction.Normalize();

        /* If we have a non-zero direction then look towards that direciton otherwise do nothing */
        if (direction.sqrMagnitude > 0.001f)
        {
            /* Mulitply by -1 because counter clockwise on the y-axis is in the negative direction */
            float toRotation = -1 * (Mathf.Atan2(direction.z, direction.x) * Mathf.Rad2Deg);
            float rotation = Mathf.LerpAngle(locomotion.Rotation.eulerAngles.y, toRotation, Time.deltaTime * turnSpeed);

            locomotion.Rotation = Quaternion.Euler(0, rotation, 0);
        }
    }

    public void LookAtDirection(Quaternion toRotation)
    {
        LookAtDirection(toRotation.eulerAngles.y);
    }

    /// <summary>
    /// Makes the character's rotation lerp closer to the given target rotation (in degrees).
    /// </summary>
    /// <param name="toRotation">the desired rotation to be looking at in degrees</param>
    public void LookAtDirection(float toRotation)
    {
        float rotation = Mathf.LerpAngle(locomotion.Rotation.eulerAngles.y, toRotation, Time.deltaTime * turnSpeed);
        locomotion.Rotation = Quaternion.Euler(0, rotation, 0);
    }

    /// <summary>
    /// Returns the steering for a character so it arrives at the target
    /// </summary>
    public Vector3 Arrive(Vector3 targetPosition)
    {
        Debug.DrawLine(transform.position, targetPosition, Color.cyan, 0f, false);

        targetPosition.y = 0;

        /* Get the right direction for the linear acceleration */
        Vector3 targetVelocity = targetPosition - locomotion.Position;
        //Debug.Log("Displacement " + targetVelocity.ToString("f4"));

        /* Get the distance to the target */
        float dist = targetVelocity.magnitude;

        /* If we are within the stopping radius then stop */
        if (dist < targetRadius)
        {
            locomotion.Velocity = Vector3.zero;
            return Vector3.zero;
        }

        /* Calculate the target speed, full speed at slowRadius distance and 0 speed at 0 distance */
        float targetSpeed;
        if (dist > slowRadius)
        {
            targetSpeed = maxVelocity;
        }
        else
        {
            targetSpeed = maxVelocity * (dist / slowRadius);
        }

        /* Give targetVelocity the correct speed */
        targetVelocity.Normalize();
        targetVelocity *= targetSpeed;

        /* Calculate the linear acceleration we want */
        Vector3 acceleration = targetVelocity - locomotion.Velocity;
        /* Rather than accelerate the character to the correct speed in 1 second, 
         * accelerate so we reach the desired speed in timeToTarget seconds 
         * (if we were to actually accelerate for the full timeToTarget seconds). */
        acceleration *= 1 / timeToTarget;

        /* Make sure we are accelerating at max acceleration */
        if (acceleration.magnitude > maxAcceleration)
        {
            acceleration.Normalize();
            acceleration *= maxAcceleration;
        }

        //Debug.Log("Velocity CC: " + pm.Velocity.magnitude);
        return acceleration;
    }

    public Vector3 Interpose(CharacterLocomotion target1, CharacterLocomotion target2)
    {
        Vector3 midPoint = (target1.Position + target2.Position) / 2;

        float timeToReachMidPoint = Vector3.Distance(midPoint, transform.position) / maxVelocity;

        Vector3 futureTarget1Pos = target1.Position + target1.Velocity * timeToReachMidPoint;
        Vector3 futureTarget2Pos = target2.Position + target2.Velocity * timeToReachMidPoint;

        midPoint = (futureTarget1Pos + futureTarget2Pos) / 2;

        return Arrive(midPoint);
    }

    /// <summary>
    /// Checks to see if the target is in front of the character
    /// </summary>
    public bool IsInFront(Vector3 target)
    {
        return IsFacing(target, 0);
    }

    public bool IsFacing(Vector3 target, float cosineValue)
    {
        Vector3 facing = transform.right.normalized;

        Vector3 directionToTarget = (target - transform.position);
        directionToTarget.Normalize();

        return Vector3.Dot(facing, directionToTarget) >= cosineValue;
    }

    /// <summary>
    /// Returns the given orientation (in radians) as a unit vector
    /// </summary>
    /// <param name="orientation">the orientation in radians</param>
    /// <param name="is3DGameObj">is the orientation for a 3D game object or a 2D game object</param>
    /// <returns></returns>
    public static Vector3 OrientationToVector(float orientation, bool is3DGameObj)
    {
        if (is3DGameObj)
        {
            /* Mulitply the orientation by -1 because counter clockwise on the y-axis is in the negative
             * direction, but Cos And Sin expect clockwise orientation to be the positive direction */
            return new Vector3(Mathf.Cos(-orientation), 0, Mathf.Sin(-orientation));
        }
        else
        {
            return new Vector3(Mathf.Cos(orientation), Mathf.Sin(orientation), 0);
        }
    }

    /// <summary>
    /// Gets the orientation of a vector as radians. For 3D it gives the orienation around the Y axis.
    /// For 2D it gaves the orienation around the Z axis.
    /// </summary>
    /// <param name="direction">the direction vector</param>
    /// <param name="is3DGameObj">is the direction vector for a 3D game object or a 2D game object</param>
    /// <returns>orientation in radians</returns>
    public static float VectorToOrientation(Vector3 direction, bool is3DGameObj)
    {
        if (is3DGameObj)
        {
            /* Mulitply by -1 because counter clockwise on the y-axis is in the negative direction */
            return -1 * Mathf.Atan2(direction.z, direction.x);
        }
        else
        {
            return Mathf.Atan2(direction.y, direction.x);
        }
    }

    /// <summary>
    /// Creates a debug cross at the given position in the scene view to help with debugging.
    /// </summary>
    public static void DebugCross(Vector3 position, float size = 0.5f, Color color = default(Color), float duration = 0f, bool depthTest = true)
    {
        Vector3 xStart = position + Vector3.right * size * 0.5f;
        Vector3 xEnd = position - Vector3.right * size * 0.5f;

        Vector3 yStart = position + Vector3.up * size * 0.5f;
        Vector3 yEnd = position - Vector3.up * size * 0.5f;

        Vector3 zStart = position + Vector3.forward * size * 0.5f;
        Vector3 zEnd = position - Vector3.forward * size * 0.5f;

        Debug.DrawLine(xStart, xEnd, color, duration, depthTest);
        Debug.DrawLine(yStart, yEnd, color, duration, depthTest);
        Debug.DrawLine(zStart, zEnd, color, duration, depthTest);
    }
}
