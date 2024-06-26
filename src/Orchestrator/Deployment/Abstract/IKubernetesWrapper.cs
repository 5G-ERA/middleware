using Middleware.Common.Result;
using Middleware.Orchestrator.Models;

namespace Middleware.Orchestrator.Deployment;

public interface IKubernetesWrapper
{
    Task<List<string>> GetCurrentlyDeployedNetApps();
    /// <summary>
    /// Deploy specified NetApp pari (deployment and service)
    /// </summary>
    /// <param name="netApp"></param>
    /// <returns>Result if the deployment operation has succeeded</returns>
    Task<Result> DeployNetApp(DeploymentPair netApp);

    /// <summary>
    ///     Deletes the instance specified. The deletion includes associated deployment and service
    /// </summary>
    /// <param name="instanceId"></param>
    /// <returns></returns>
    Task<Result> TerminateNetAppById(Guid instanceId);

    /// <summary>
    ///     Terminates the Inter Relay NetApp based on the Action Plan Id and Action Id
    /// </summary>
    /// <param name="labels">Kubernetes labels used to identify the realy</param>
    /// <returns>Name of the deleted relay if deleted</returns>
    Task<Result<string>> TerminateInterRelayNetApp(Dictionary<string, string> labels);
}