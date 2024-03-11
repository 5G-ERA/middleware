using Middleware.Common.Result;
using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Deployment;

public interface IDeploymentService
{
    /// <summary>
    ///     Deletes the ActionPlan and resources instantiated by the specified ActionPlan if not used by other ActionPlans
    /// </summary>
    /// <param name="actionPlan">Action Plan to be deleted</param>
    /// <returns>Has the operation succeeded</returns>
    Task<Result> DeletePlanAsync(ActionPlanModel actionPlan);

    Task DeleteActionAsync(Guid actionPlanId, Guid actionId);

    Task DeployActionAsync(Guid actionPlanId, Guid actionId);
    Task<Result> DeployActionPlanAsync(TaskModel task, Guid robotId);
}