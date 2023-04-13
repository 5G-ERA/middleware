using Middleware.Models.Domain;

namespace Middleware.TaskPlanner.Services;

public interface IPublishService
{
    /// <summary>
    /// Publishes prepared action plan to the Orchestrator
    /// </summary>
    /// <param name="task"></param>
    /// <param name="robot"></param>
    /// <returns></returns>
    Task PublishPlanAsync(TaskModel task, RobotModel robot);
    /// <summary>
    /// Publishes the request to delete instance in local middleware as part of the switchover process
    /// </summary>
    /// <param name="actionPlanId">Unique identifier of ActionPlan</param>
    /// <param name="actionId">Unique identifier of the deployed instance</param>
    /// <returns></returns>
    Task PublishSwitchoverDeleteInstance(Guid actionPlanId, Guid actionId);
    /// <summary>
    /// Publishes the request to deploy the instance into the remote middleware as part of the switchover process
    /// </summary>
    /// <param name="actionPlanId">Unique identifier of ActionPlan</param>
    /// <param name="actionId">Unique identifier of the deployed instance</param>
    /// <param name="location">Name of the remote middleware location</param>
    /// <param name="locationType">Type of the remote middleware location</param>
    /// <returns></returns>
    Task PublishSwitchoverDeployInstance(Guid actionPlanId, Guid actionId, string location, string locationType);
}