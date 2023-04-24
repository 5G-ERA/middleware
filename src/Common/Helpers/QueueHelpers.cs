namespace Middleware.Common.Helpers;

/// <summary>
/// Class responsible for handling common 
/// </summary>
public static class QueueHelpers
{
    /// <summary>
    /// Constructs the routing key based on the deployment properties 
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

    private static string GetQueueName(string organization, string instanceName, string queueName)
    {
        return $"{organization}-{instanceName}-{queueName}";
    }
    /// <summary>
    /// Constructs the deployment queue name for this specific Middleware instance
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
    /// Constructs the switchover delete queue name for this specific Middleware instance
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
    /// Constructs the switchover deployment queue name for this specific Middleware instance
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
}