using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common;
using Middleware.Common.Attributes;
using Middleware.Common.Enums;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Mappings;
using Middleware.RedisInterface.Requests;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class ActionController : MiddlewareController
{
    private readonly IActionRepository _actionRepository;
    private readonly IActionPlanRepository _actionPlanRepository;
    private readonly ILogger _logger;
    private readonly IActionService _actionService;

    public ActionController(IActionRepository actionRepository,
        IActionPlanRepository actionPlanRepository,
        ILogger<ActionController> logger,
        IActionService actionService)
    {
        _actionRepository = actionRepository ?? throw new ArgumentNullException(nameof(actionRepository));
        _actionPlanRepository = actionPlanRepository ?? throw new ArgumentNullException(nameof(actionPlanRepository));
        _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        _actionService = actionService;
    }

    /// <summary>
    /// Get all the actions
    /// </summary>
    /// <returns> the list of ActionModel entities </returns>
    [HttpGet(Name = "ActionGetAll")]
    [ProducesResponseType(typeof(GetActionsResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetAllAsync()
    {
        try
        {
            List<ActionModel> models = await _actionRepository.GetAllAsync();
            if (models.Any() == false)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, "action", "No actions were found.");
            }
            return Ok(models.ToActionsResponse());
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }
    
    /// <summary>
    /// Get an action entity by id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> the ActionModel entity for the specified id </returns>
    [HttpGet]
    [Route("{id}", Name = "ActionGetById")]
    [ProducesResponseType(typeof(ActionResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetByIdAsync(Guid id)
    {
        try
        {
            ActionModel model = await _actionService.GetByIdAsync(id);
            if (model == null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), $"Action with id: '{id}' was not found.");
            }
            return Ok(model.ToActionResponse());
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    /// Add a new action entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the newly created ActionModel entity </returns>
    [HttpPost(Name = "ActionAdd")]
    [ProducesResponseType(typeof(ActionResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddAsync([FromBody] ActionRequest request)
    {
        try
        {
            var existingTask = await _actionRepository.FindSingleAsync(t=>t.Name == request.Name);
            if (existingTask is not null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, "name", "Action with specified name already exists");
            }
            var action  = await _actionService.AddAsync(request.ToAction());
            return Ok(action.ToActionResponse());
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    /// Update an existing action entity
    /// </summary>
    /// <returns> the modified ActionModel entity </returns>
    [HttpPut]
    [Route("{id}", Name = "ActionUpdate")]
    [ProducesResponseType(typeof(ActionResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> UpdateActionAsync([FromMultiSource] UpdateActionRequest request)
    {
        try
        {
            var existingAction = await _actionService.GetByIdAsync(request.Id);
            if (existingAction is null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request), "Object to be updated was not found.");
            }

            var action = request.ToAction();
            await _actionService.UpdateAsync(action);
            
            var response = action.ToActionResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    /// Delete an action entity with the given id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> no return </returns>
    [HttpDelete]
    [Route("{id}", Name = "ActionDelete")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteByIdAsync(Guid id)
    {
        try
        {
            var exists = await _actionService.GetByIdAsync(id);
            if (exists is null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), "The specified Action was not found.");
            }
            await _actionService.DeleteAsync(id);
            return Ok();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    /// Creates a new relation between two models
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("AddRelation", Name = "ActionAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddRelationAsync([FromBody] RelationModel request)
    {
        if (request == null)
        {
            return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request), "The request body was not specified.");
        }
        try
        {
            bool isValid = await _actionRepository.AddRelationAsync(request);
            if (!isValid)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "The relation was not created");
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
        return Ok(request);
    }

    /// <summary>
    /// Retrieves a single relation by name
    /// </summary>
    /// <param name="id"></param>
    /// <param name="name"></param>
    /// <param name="direction"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("relation/{name}", Name = "ActionGetRelationByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationAsync(Guid id, string name, string direction = "Outgoing") //Guid of node and name of relationship
    {

        if (string.IsNullOrWhiteSpace(name))
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(name), "Relation name not specified");
        }
        if (id == Guid.Empty)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), "Relation ID not specified");
        }

        if (Enum.TryParse<RelationDirection>(direction, out var directionEnum) == false)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(direction), "Wrong Relation direction specified");
        }
        var inputDirection = directionEnum;
        try
        {
            var relations = await _actionRepository.GetRelation(id, name, inputDirection);
            if (!relations.Any())
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, "relation", "Relations were not found.");
            }
            return Ok(relations);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    /// Retrieves two relations by their names
    /// </summary>
    /// <param name="id"></param>
    /// <param name="firstName"></param>
    /// <param name="secondName"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("relations/{firstName}/{secondName}", Name = "ActionGetRelationsByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
    {
        try
        {
            List<string> relationNames = new List<string>() { firstName, secondName };
            var relations = await _actionRepository.GetRelations(id, relationNames);
            if (!relations.Any())
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, "relation", "Relations were not found.");
            }
            return Ok(relations);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    #region ActionPlan

    /// <summary>
    /// Retrieves all ActionPlans
    /// </summary>
    /// <returns></returns>
    [HttpGet]
    [Route("plan", Name = "ActionPlanGetAll")]
    [ProducesResponseType(typeof(List<ActionPlanModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetAllActionPlansAsync()
    {
        try
        {
            var plans = await _actionPlanRepository.GetAllAsync();
            if (plans.Any() == false)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, "actionPlan", "No plans have been found.");
            }
            return Ok(plans);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    /// Retrieves an ActionPlan by id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("plan/{id:guid}", Name = "ActionPlanGetById")]
    [ProducesResponseType(typeof(ActionPlanModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetActionPlanByIdAsync(Guid id)
    {
        try
        {
            var plan = await _actionPlanRepository.GetByIdAsync(id);
            if (plan == null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), "Specified plan was not found.");
            }
            return Ok(plan);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    /// Creates new ActionPlan
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("plan", Name = "ActionPlanAdd")]
    [ProducesResponseType(typeof(ActionPlanModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddActionPlanAsync(ActionPlanModel request)
    {
        try
        {
            if (request == null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request), "Plan must be specified");
            }
            var plan = await _actionPlanRepository.AddAsync(request, () => request.Id);
            return Ok(plan);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    /// Delete ActionPlan by id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("plan/{id}", Name = "ActionPlanDelete")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteActionPlanAsync(Guid id)
    {
        try
        {
            var deleted = await _actionPlanRepository.DeleteByIdAsync(id);
            if (deleted == false)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), "The specified plan was not found.");
            }
            return Ok();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    /// Patches an existing ActionPlan by id
    /// </summary>
    /// <param name="id"></param>
    /// <param name="actionPlan"></param>
    /// <returns></returns>
    [HttpPut]
    [Route("plan/{id:guid}", Name = "ActionPlanPatch")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> PatchActionPlanAsync(Guid id, [FromBody] ActionPlanModel actionPlan)
    {
        try
        {
            if (actionPlan == null || id == Guid.Empty)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), "Id or updated object has not been specified");
            }
            var deleted = await _actionPlanRepository.DeleteByIdAsync(id);
            if (deleted == false)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), "The specified plan was not found.");
            }

            var updatedPlan = await _actionPlanRepository.AddAsync(actionPlan, () => id);
            return Ok(updatedPlan);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }
    /// <summary>
    /// Get action plan given robot Id.
    /// </summary>
    [HttpGet]
    [Route("plan/robot/{id}", Name = "GetActionPlanByRobotIdAsync")]
    [ProducesResponseType(typeof(List<ActionPlanModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetActionPlanByRobotIdAsync(Guid id)
    {
        try
        {
            if (id == Guid.Empty)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), "Robot id was not specified");
            }
            // Get list of actionPlans from specific robotId.
            List<ActionPlanModel> actionPlans = await _actionPlanRepository.GetRobotActionPlans(id);
            if (actionPlans.Any() == false)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), "No ActionPlans for robot were found");
            }
            //List<ActionPlanModel> activePoliciesRecords = actionPlans.Select(p => new ActionPlanModel(p.Id, p.Name, p.Description)).ToList();
            return Ok(actionPlans);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }
    #endregion

    /// <summary>
    /// Get latest action plan given robot Id.
    /// </summary>
    [HttpGet]
    [Route("plan/robot/{id}/latest", Name = "GetLatestActionPlanByRobotIdAsync")]
    [ProducesResponseType(typeof(ActionPlanModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetLatestActionPlanByRobotIdAsync(Guid id)
    {
        try
        {
            if (id == Guid.Empty)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), "Robot id was not specified");
            }
            // Get list of actionPlans from specific robotId.
            List<ActionPlanModel> actionPlans = await _actionPlanRepository.GetRobotActionPlans(id);
            if (actionPlans.Any() == false)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), "No ActionPlans for robot were found");
            }
            //Get the newest task of robot.
            Dictionary<ActionPlanModel, DateTime> tempDic = new Dictionary<ActionPlanModel, DateTime>();
            Dictionary<ActionPlanModel, DateTime> orderedTempDic = new Dictionary<ActionPlanModel, DateTime>();

            // Complete tempDic
            foreach (ActionPlanModel plan in actionPlans)
            {
                DateTime d;
                DateTime.TryParseExact(plan.Status, "ggyyyy$dd-MMM (dddd)", System.Globalization.CultureInfo.InvariantCulture, System.Globalization.DateTimeStyles.None, out d);
                tempDic.Add(plan, d);
            }

            // Order a new dictionary
            foreach (KeyValuePair<ActionPlanModel, DateTime> pair in tempDic.OrderByDescending(p => p.Value))
            {
                orderedTempDic.Add(pair.Key, pair.Value);
            }
            ActionPlanModel last = orderedTempDic.Keys.FirstOrDefault();
            return Ok(last);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

}