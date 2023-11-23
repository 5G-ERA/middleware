using System.ComponentModel;
using Middleware.Models.Enums;

namespace Middleware.Common.Helpers;

/// <summary>
///     Class responsible for handling common
/// </summary>
public static class QueueHelpers
{
    /// <summary>
    ///     Constructs the routing key based on the deployment properties
    /// </summary>
    /// <param name="instanceName"></param>
    /// <param name="instanceType"></param>
    /// <returns></returns>
    public static string ConstructRoutingKey(string instanceName, string instanceType)
    {
        if (instanceName == null) throw new ArgumentNullException(nameof(instanceName));
        if (instanceType == null) throw new ArgumentNullException(nameof(instanceType));

        return $"{instanceName}-{instanceType}";
    }

    /// <summary>
    ///     Constructs the routing key based on the deployment properties
    /// </summary>
    /// <param name="instanceName"></param>
    /// <param name="locationType"></param>
    /// <returns></returns>
    public static string ConstructRoutingKey(string instanceName, LocationType locationType)
    {
        if (instanceName == null) throw new ArgumentNullException(nameof(instanceName));
        if (!Enum.IsDefined(typeof(LocationType), locationType))
            throw new InvalidEnumArgumentException(nameof(locationType), (int)locationType, typeof(LocationType));

        return ConstructRoutingKey(instanceName, locationType.ToString());
    }

    private static string GetQueueName(string organization, string instanceName, string queueName)
    {
        return $"{organization}-{instanceName}-{queueName}";
    }

    /// <summary>
    ///     Constructs the deployment queue name for this specific Middleware instance
    /// </summary>
    /// <param name="organization"></param>
    /// <param name="instanceName"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentNullException"></exception>
    public static string ConstructDeploymentQueueName(string organization, string instanceName)
    {
        if (organization == null) throw new ArgumentNullException(nameof(organization));
        if (instanceName == null) throw new ArgumentNullException(nameof(instanceName));

        return GetQueueName(organization, instanceName, "deployments");
    }

    /// <summary>
    ///     Constructs the switchover delete queue name for this specific Middleware instance
    /// </summary>
    /// <param name="organization"></param>
    /// <param name="instanceName"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentNullException">When parameters are not specified or contain empty or whitespace string</exception>
    public static string ConstructSwitchoverDeleteActionQueueName(string organization, string instanceName)
    {
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(organization));
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(instanceName));

        return GetQueueName(organization, instanceName, "switchover-action-delete");
    }

    /// <summary>
    ///     Constructs the name of the queue that will be used to create a new YARP dynamic route
    /// </summary>
    /// <param name="organization"></param>
    /// <param name="instanceName"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentNullException">When parameters are not specified or contain empty or whitespace string</exception>
    public static string ConstructGatewayAddNetAppEntryMessageQueueName(string organization, string instanceName)
    {
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(organization));
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(instanceName));

        return GetQueueName(organization, instanceName, "gateway-add-entry");
    }


    /// <summary>
    ///     Constructs the name of the queue that will be used to delete the YARP dynamic route
    /// </summary>
    /// <param name="organization"></param>
    /// <param name="instanceName"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentNullException">When parameters are not specified or contain empty or whitespace string</exception>
    public static string ConstructGatewayDeleteNetAppEntryMessageQueueName(string organization, string instanceName)
    {
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(organization));
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(instanceName));

        return GetQueueName(organization, instanceName, "gateway-delete-entry");
    }

    /// <summary>
    ///     Constructs the switchover deployment queue name for this specific Middleware instance
    /// </summary>
    /// <param name="organization"></param>
    /// <param name="instanceName"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentNullException">When parameters are not specified or contain empty or whitespace string</exception>
    public static string ConstructSwitchoverDeployActionQueueName(string organization, string instanceName)
    {
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(organization));
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(instanceName));

        return GetQueueName(organization, instanceName, "switchover-action-deploy");
    }

    /// <summary>
    ///     Constructs the switchover deployment queue name for this specific Middleware instance
    /// </summary>
    /// <param name="organization"></param>
    /// <param name="instanceName"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentNullException">When parameters are not specified or contain empty or whitespace string</exception>
    public static string ConstructSliceImsiConnectionQueueName(string organization, string instanceName)
    {
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(organization));
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(instanceName));

        return GetQueueName(organization, instanceName, "slice-imsi-connect");
    }

    /// <summary>
    ///     Constructs the resource planning queue name
    /// </summary>
    /// <param name="organization"></param>
    /// <param name="instanceName"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentNullException">When parameters are not specified or contain empty or whitespace string</exception>
    public static string ConstructResourcePlanningServiceQueueName(string organization, string instanceName)
    {
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(organization));
        if (string.IsNullOrWhiteSpace(organization)) throw new ArgumentNullException(nameof(instanceName));

        return GetQueueName(organization, instanceName, "resource-plan");
    }
}