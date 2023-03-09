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

        return $"{organization}-{instanceName}-deployments";
    }
}