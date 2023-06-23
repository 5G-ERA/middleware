using k8s.Models;

namespace Middleware.Orchestrator.Deployment;

internal interface IRosConnectionBuilder
{
    /// <summary>
    ///     Enables the communication using ROS between the services within deployment
    /// </summary>
    /// <param name="dpl"></param>
    /// <returns></returns>
    V1Deployment EnableRosCommunication(V1Deployment dpl);
}