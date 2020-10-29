using UnityEngine;


public class PursueUnitCC : MonoBehaviour
{
    public CharacterLocomotion target;

    SteeringBasicsCC steeringBasics;
    
    void Start()
    {
        steeringBasics = GetComponent<SteeringBasicsCC>();
    }

    void FixedUpdate()
    {
        Vector3 accel = steeringBasics.Pursure(target);

        steeringBasics.Steer(accel);
        steeringBasics.LookWhereYoureGoing();
    }
}
