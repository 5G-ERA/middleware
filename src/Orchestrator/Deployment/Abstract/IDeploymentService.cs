using k8s.Models;
using Middleware.Common.Enums;
using Middleware.Common.Structs;
using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Deployment;

public interface IDeploymentService
{
    /// <summary>
    ///     Creates startup deployment needed to instantiate the middleware.
    /// </summary>
    /// <param name="name">Name of the Middleware component to be deployed</param>
    /// <param name="tag">Tag of the image to be used</param>
    /// <returns>Deployment for the middleware component. Created deployment is not instantiated</returns>
    V1Deployment CreateStartupDeployment(string name, string tag);

    /// <summary>
    ///     Creates the service of the specified type with the metadata
    /// </summary>
    /// <param name="serviceImageName">Name of the service to be deployed</param>
    /// <param name="kind">Kind of the service to be deployed</param>
    /// <param name="meta">Metadata for the service from the existing deployment</param>
    /// <param name="nodePort"></param>
    /// <returns>Service of the specified type. Service has not been deployed</returns>
    V1Service CreateStartupService(string serviceImageName, K8SServiceKind kind, V1ObjectMeta meta, int? nodePort = null);

    /// <summary>
    ///     Deletes the ActionPlan and resources instantiated by the specified ActionPlan if not used by other ActionPlans
    /// </summary>
    /// <param name="actionPlan">Action Plan to be deleted</param>
    /// <returns>Has the operation succeeded</returns>
    Task<bool> DeletePlanAsync(ActionPlanModel actionPlan);

    Task DeleteActionAsync(Guid actionPlanId, Guid actionId);

    Task DeployActionAsync(Guid actionPlanId, Guid actionId);
    Task<Result<bool>> DeployActionPlanAsync(TaskModel task, Guid robotId);
}