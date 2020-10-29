using UnityEngine;
using System.Collections.Generic;


public class NearSensorCC : MonoBehaviour
{

    CharacterLocomotion parent;
    HashSet<CharacterLocomotion> _targets = new HashSet<CharacterLocomotion>();

    private void Awake()
    {
        parent = GetComponentInParent<CharacterLocomotion>();
    }

    public HashSet<CharacterLocomotion> targets
    {
        get
        {
            /* Remove any MovementAIRigidbodies that have been destroyed */
            _targets.RemoveWhere(IsNull);
            return _targets;
        }
    }

    static bool IsNull(CharacterLocomotion r)
    {
        return (r == null || r.Equals(null));
    }

    void TryToAdd(Component other)
    {
        CharacterLocomotion rb = other.GetComponent<CharacterLocomotion>();

        if (rb == parent)
            return;

        if (rb != null)
        {
            _targets.Add(rb);
        }
    }

    void TryToRemove(Component other)
    {
        CharacterLocomotion rb = other.GetComponent<CharacterLocomotion>();
        if (rb != null)
        {
            _targets.Remove(rb);
        }
    }

    void OnTriggerEnter(Collider other)
    {
        TryToAdd(other);
    }

    void OnTriggerExit(Collider other)
    {
        TryToRemove(other);
    }
}
