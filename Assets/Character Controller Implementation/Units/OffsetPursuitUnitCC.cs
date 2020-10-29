using UnityEngine;

public class OffsetPursuitUnitCC : MonoBehaviour
{
    public CharacterLocomotion target;

    public Vector3 offset;
    public float groupLookDist = 1.5f;

    SteeringBasicsCC steeringBasics;
    //OffsetPursuitCC offsetPursuit;
    //SeparationCC separation;

    NearSensorCC sensor;

    void Start()
    {
        steeringBasics = GetComponent<SteeringBasicsCC>();
        //offsetPursuit = GetComponent<OffsetPursuitCC>();
        //separation = GetComponent<SeparationCC>();

        sensor = transform.Find("SeparationSensor").GetComponent<NearSensorCC>();
    }

    void FixedUpdate()
    {
        Vector3 targetPos;
        Vector3 offsetAccel = steeringBasics.OffsetPursuit(target, offset, out targetPos);
        Vector3 sepAccel = steeringBasics.Separation(sensor.targets);

        steeringBasics.Steer(offsetAccel + sepAccel);

        /* If we are still arriving then look where we are going, else look the same direction as our formation target */
        if (Vector3.Distance(transform.position, targetPos) > groupLookDist)
        {
            steeringBasics.LookWhereYoureGoing();
        }
        else
        {
            steeringBasics.LookAtDirection(target.Rotation);
        }
    }
}