using UnityEngine;
using UnityMovementAI;

public class ColAvoidUnitCC : MonoBehaviour
{
    public Transform finalTarget;
    public CharacterLocomotion[] AvoidUnits;

    SteeringBasicsCC steeringBasics;

    void Start()
    {
        steeringBasics = GetComponent<SteeringBasicsCC>();
    }

    void FixedUpdate()
    {
        Vector3 seek = steeringBasics.Seek(finalTarget.position);
        Vector3 accel = steeringBasics.CollisionAvoidance(AvoidUnits);

        steeringBasics.Steer(seek + accel);
        steeringBasics.LookWhereYoureGoing();
    }
}
