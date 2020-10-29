using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SeekUnitCC : MonoBehaviour
{
    public Transform target;

    SteeringBasicsCC steeringBasics;

    void Start()
    {
        steeringBasics = GetComponent<SteeringBasicsCC>();
    }

    void FixedUpdate()
    {
        Vector3 accel = steeringBasics.Seek(target.position);

        steeringBasics.Steer(accel);
        steeringBasics.LookWhereYoureGoing();
    }
}
