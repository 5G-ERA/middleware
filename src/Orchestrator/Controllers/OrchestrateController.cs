using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Deployment;
using Middleware.RedisInterface.Sdk;
using Middleware.RedisInterface.Contracts.Mappings;

namespace Middleware.Orchestrator.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class OrchestrateController : Controller
{
    private readonly IDeploymentService _deploymentService;
    private readonly ILogger _logger;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public OrchestrateController(IDeploymentService deploymentService,
        ILogger<OrchestrateController> logger,
        IRedisInterfaceClient redisInterfaceClient)
    {
        _deploymentService = deploymentService;
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
    }

    public record OrchestratorResourceInput(TaskModel Task, RobotModel Robot);

    /// <summary>
    /// Request orchestration of the resources defied in the plan
    /// </summary>
    /// <param name="task"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("plan", Name = "InstantiateNewPlan")]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> InstantiateNewPlan([FromBody] OrchestratorResourceInput task)
    {
        int statusCode = (int)HttpStatusCode.BadRequest;
        if (task is null)
        {
            return BadRequest("Plan is not specified");
        }

        try
        {
            statusCode = (int)HttpStatusCode.OK;
            var result = await _deploymentService.DeployAsync(task.Task, task.Robot.Id);
            if (result == false)
            {
                statusCode = (int)HttpStatusCode.InternalServerError;
                //TODO: provide more detailed information on what went wrong with the deployment of the task
                return StatusCode(statusCode,
                    new ApiResponse(statusCode, "There was a problem while deploying the task instance"));
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "There was a problem while deploying the task instance: {task}", task);
            statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode,
                new ApiResponse(statusCode, $"There was a problem while deploying the task instance: {ex.Message}"));
        }

        return Ok(task);
    }

    /// <summary>
    /// Get the action plan by the ActionPlanId identifier 
    /// </summary>
    /// <param name="id">Identifier of the created ActionPlan</param>
    /// <returns></returns>
    [HttpGet]
    [Route("plan/{id}", Name = "GetActionPlanByPlanId")]
    [ProducesResponseType(typeof(ActionPlanModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<ActionPlanModel>> GetActionsByPlanId(Guid id)
    {
        int statusCode;
        try
        {
            ActionPlanModel actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(id);
            if (actionPlan is null)
            {
                statusCode = (int)HttpStatusCode.NotFound;
                return NotFound(new ApiResponse(statusCode, "Specified action plan with specified id not found"));
            }

            //TODO: retrieve the latest info about the plan?
            return Ok(actionPlan);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "There was an error while retrieving the action plan information");
            statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode,
                new ApiResponse(statusCode,
                    $"There was an error while retrieving the action plan information: {ex.Message}"));
        }
    }

    /// <summary>
    /// Request orchestration of the resources defied in the plan
    /// </summary>
    /// <param name="task"></param>
    /// <returns></returns>
    [HttpPatch]
    [Route("plan", Name = "UpdatePlan")]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    public async Task<IActionResult> UpdatePlan([FromBody] TaskModel task)
    {
        //TODO: redeploy services for new plan
        return Ok(task);
    }

    /// <summary>
    /// Deletes the instances instantiated with the specified action 
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("action/{id}", Name = "DeleteActionById")]
    [ProducesResponseType((int)HttpStatusCode.OK)]
    [ProducesResponseType((int)HttpStatusCode.NotFound)]
    public async Task<IActionResult> DeleteActionById(Guid id)
    {
        // TODO: Delete action with specified Id
        return Ok();
    }

    /// <summary>
    /// Delete plan by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("plan/{id}", Name = "DeletePlanById")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult> DeletePlanById(Guid id)
    {
        if (id == Guid.Empty)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Id of the plan  has to be specified"));
        }

        try
        {
            ActionPlanModel actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(id);
            if (actionPlan is null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                    $"Could not find the Action Plan with specified id: {id}"));
            }

            var isSuccess = await _deploymentService.DeletePlanAsync(actionPlan);

            //Delete the LOCATED_AT relationships between instance and edge/cloud.
            List<ActionModel> actionTempList = actionPlan.ActionSequence;
            foreach (ActionModel action in actionTempList)
            {
                BaseModel placement;
                RelationModel tempRelationModel = new RelationModel();
                tempRelationModel.RelationName = "LOCATED_AT";

                if (action.Placement!.ToUpper().Contains("CLOUD"))
                {
                    var cloud = (await _redisInterfaceClient.GetCloudByNameAsync(action.Placement))?.ToCloud();
                    placement = cloud;

                }
                else
                {
                    var edge = (await _redisInterfaceClient.GetEdgeByNameAsync(action.Placement)).ToEdge(); 
                    placement = edge;
                }

                foreach (InstanceModel instance in action.Services)
                {
                    GraphEntityModel tempInstanceGraph = new GraphEntityModel(); //The instances that the action need.
                    tempInstanceGraph.Name = instance.Name;
                    tempInstanceGraph.Id = instance.Id;
                    tempInstanceGraph.Type = "INSTANCE";

                    //delete all the located_at relationships between all instances of 1 action and the resources been edge/cloud
                    await _redisInterfaceClient.DeleteRelationAsync(instance, placement, "LOCATED_AT");
                }
            }

            // //Delete the relationship OWNS between the robot and the task that has been completed.
            // RelationModel deleteRelationRobotOwnsTask = new RelationModel();
            // deleteRelationRobotOwnsTask.RelationName = "OWNS";

            RobotModel tempRobotObject = (await _redisInterfaceClient.RobotGetByIdAsync(actionPlan.RobotId))?.ToRobot();
            //
            // GraphEntityModel tempRobotGraph = new GraphEntityModel();
            // tempRobotGraph.Id = tempRobotObject.Id;
            // tempRobotGraph.Name = tempRobotObject.Name;

            TaskModel tempTaskObject = (await _redisInterfaceClient.TaskGetByIdAsync(actionPlan.TaskId))?.ToTask();
            //
            // GraphEntityModel tempTaskGraph = new GraphEntityModel();
            // tempTaskGraph.Id = id;
            // tempTaskGraph.Name = tempTaskObject.Name;
            //
            // deleteRelationRobotOwnsTask.InitiatesFrom = tempRobotGraph;
            // deleteRelationRobotOwnsTask.PointsTo = tempTaskGraph;

            await _redisInterfaceClient.DeleteRelationAsync(tempRobotObject, tempTaskObject, "OWNS");

            if (isSuccess == false)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                return StatusCode(statusCode,
                    new ApiResponse(statusCode, $"Unable to delete the services for the action plan with id {id}"));
            }

            await _redisInterfaceClient.ActionPlanDeleteAsync(id);
            return Ok();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error while deleting the specified action plan with id {id}", id);
            int statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode,
                new ApiResponse(statusCode, $"Could not delete the action plan with id: {id}"));
        }
    }


    /// <summary>
    /// Instantiate the resources for specified actions
    /// </summary>
    /// <param name="actions">List of actions to be instantiated</param>
    /// <returns>Http Status code and List of instantiated services</returns>
    [HttpPost]
    [Route("execute")]
    [ProducesResponseType(typeof(List<InstanceModel>), (int)HttpStatusCode.OK)]
    public async Task<IActionResult> InstantiateResources([FromBody] List<ActionModel> actions)
    {
        //TODO: instantiate services for action
        return Ok(new List<InstanceModel>());
    }
}