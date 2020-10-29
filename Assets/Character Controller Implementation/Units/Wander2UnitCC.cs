using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wander2UnitCC : MonoBehaviour
{
    SteeringBasicsCC steeringBasics;

    void Start()
    {
        steeringBasics = GetComponent<SteeringBasicsCC>();
    }

    void FixedUpdate()
    {
        Vector3 accel = steeringBasics.Wander();

        steeringBasics.Steer(accel);
        steeringBasics.LookWhereYoureGoing();
    }
}