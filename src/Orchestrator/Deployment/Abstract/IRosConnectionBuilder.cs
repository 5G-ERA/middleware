using k8s.Models;
using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Deployment;

internal interface IRosConnectionBuilder
{
    /// <summary>
    ///     Ros Version the builder prepares connection method
    /// </summary>
    int RosVersion { get; }

    /// <summary>
    ///     Ros Distro the builder prepares connection for
    /// </summary>
    string RosDistro { get; }

    /// <summary>
    ///     Enables the communication using ROS between the services within deployment
    /// </summary>
    /// <param name="dpl"></param>
    /// <param name="rosTopics"></param>
    /// <returns></returns>
    V1Deployment EnableRosCommunication(V1Deployment dpl, IReadOnlyList<RosTopicModel> rosTopics);

    /// <summary>
    ///     Enables the communication with the service to the Relay NetApp
    /// </summary>
    /// <param name="service"></param>
    /// <returns></returns>
    V1Service EnableRelayNetAppCommunication(V1Service service);
}