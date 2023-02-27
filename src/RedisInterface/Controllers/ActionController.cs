using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class ActionController : ControllerBase
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
    /// Get all the ActionModel entities
    /// </summary>
    /// <returns> the list of ActionModel entities </returns>
    [HttpGet(Name = "ActionGetAll")]
    [ProducesResponseType(typeof(List<ActionModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<IEnumerable<ActionModel>>> GetAllAsync()
    {
        try
        {

            List<ActionModel> models = await _actionRepository.GetAllAsync();
            if (models.Any() == false)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "No actions were found."));
            }
            return Ok(models);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    //New end points for depends_on property for actions.

    /// <summary>
    /// Get an ActionModel entity by id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> the ActionModel entity for the specified id </returns>
    [HttpGet]
    [Route("{id}", Name = "ActionGetById")]
    [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetByIdAsync(Guid id)
    {
        try
        {
            ActionModel model = await _actionService.GetByIdAsync(id);
            if (model == null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, $"Action with id: '{id}' was not found."));
            }
            return Ok(model);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    /// Add a new ActionModel entity
    /// </summary>
    /// <param name="model"></param>
    /// <returns> the newly created ActionModel entity </returns>
    [HttpPost(Name = "ActionAdd")]
    [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<ActionModel>> AddAsync([FromBody] ActionModel model)
    {
        if (model == null)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        }
        try
        {
            model = await _actionService.AddAsync(model);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
        return Ok(model);
    }

    /// <summary>
    /// Partially update an existing ActionModel entity
    /// </summary>
    /// <param name="patch"></param>
    /// <param name="id"></param>
    /// <returns> the modified ActionModel entity </returns>
    [HttpPatch]
    [Route("{id}", Name = "ActionPatch")]
    [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> PatchActionAsync([FromBody] ActionModel patch, [FromRoute] Guid id)
    {
        try
        {
            ActionModel model = await _actionRepository.PatchActionAsync(id, patch);
            if (model == null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object to be updated was not found."));
            }
            return Ok(model);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }


    /// <summary>
    /// Delete an ActionModel entity for the given id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> no return </returns>
    [HttpDelete]
    [Route("{id}", Name = "ActionDelete")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult> DeleteByIdAsync(Guid id)
    {
        try
        {
            var deleted = await _actionRepository.DeleteByIdAsync(id);
            if (deleted == false)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "The specified Action has not been found."));
            }
            return Ok();
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    /// Creates a new relation between two models
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("AddRelation", Name = "ActionAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<RelationModel>> AddRelationAsync([FromBody] RelationModel model)
    {
        if (model == null)
        {
            return BadRequest("Parameters were not specified.");
        }
        try
        {
            bool isValid = await _actionRepository.AddRelationAsync(model);
            if (!isValid)
            {
                return StatusCode((int)HttpStatusCode.InternalServerError, new ApiResponse((int)HttpStatusCode.InternalServerError, "The relation was not created"));
            }
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
        return Ok(model);
    }

    /// <summary>
    /// Retrieves a single relation by name
    /// </summary>
    /// <param name="id"></param>
    /// <param name="name"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("relation/{name}", Name = "ActionGetRelationByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationAsync(Guid id, string name) //Guid of node and name of relationship
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            return BadRequest(new ApiResponse((int) HttpStatusCode.BadRequest, "Relation name not specified"));
        }

        if (id == Guid.Empty)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Relation name not specified"));
        }
        try
        {
            var relations = await _actionRepository.GetRelation(id, name);
            if (!relations.Any())
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Relations were not found."));
            }
            return Ok(relations);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
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
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Relations were not found"));
            }
            return Ok(relations);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
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
            if (plans == null || plans.Any() == false)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "No plans have been found."));
            }
            return Ok(plans);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            int statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode, new ApiResponse(statusCode, ex.Message));
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
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Specified plan was not found."));
            }
            return Ok(plan);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            int statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode, new ApiResponse(statusCode, ex.Message));
        }
    }

    /// <summary>
    /// Creates new ActionPlan
    /// </summary>
    /// <param name="actionPlan"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("plan", Name = "ActionPlanAdd")]
    [ProducesResponseType(typeof(ActionPlanModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddActionPlanAsync(ActionPlanModel actionPlan)
    {
        try
        {
            if (actionPlan == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Plan cannot be null"));
            }
            var plan = await _actionPlanRepository.AddAsync(actionPlan, () => actionPlan.Id);
            if (plan == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "The specified plan has not been added."));
            }
            return Ok(plan);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
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
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "The specified plan has not been found."));
            }
            return Ok();
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
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
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Id or updated object has not been specified"));
            }
            var deleted = await _actionPlanRepository.DeleteByIdAsync(id);
            if (deleted == false)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "The specified plan has not been found."));
            }

            var updatedPlan = await _actionPlanRepository.AddAsync(actionPlan, () => id);
            return Ok(updatedPlan);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
    /// <summary>
    /// Get action plan given robot Id.
    /// </summary>
    /// <returns>List<ActionPlanModel></returns>
    [HttpGet]
    [Route("plan/robot/{robotId}", Name = "GetActionPlanByRobotIdAsync")]
    [ProducesResponseType(typeof(List<ActionPlanModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetActionPlanByRobotIdAsync(Guid robotId)
    {
        try
        {
            if (robotId == Guid.Empty)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Robot id has not been specified"));
            }
            // Get list of actionPlans from specific robotId.
            List<ActionPlanModel> actionPlans = await _actionPlanRepository.GetRobotActionPlans(robotId);
            if (actionPlans == null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));
            }
            //List<ActionPlanModel> activePoliciesRecords = actionPlans.Select(p => new ActionPlanModel(p.Id, p.Name, p.Description)).ToList();
            return Ok(actionPlans);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
    #endregion

    /// <summary>
    /// Get latest action plan given robot Id.
    /// </summary>
    /// <returns>List<ActionPlanModel></returns>
    [HttpGet]
    [Route("plan/robot/{robotId}/latest", Name = "GetLatestActionPlanByRobotIdAsync")]
    [ProducesResponseType(typeof(ActionPlanModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetLatestActionPlanByRobotIdAsync(Guid robotId)
    {
        try
        {
            if (robotId == Guid.Empty)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Robot id has not been specified"));
            }
            // Get list of actionPlans from specific robotId.
            List<ActionPlanModel> actionPlans = await _actionPlanRepository.GetRobotActionPlans(robotId);

            //Get the newest task of robot.
            Dictionary<ActionPlanModel, DateTime> tempDic = new Dictionary<ActionPlanModel, DateTime>();
            Dictionary<ActionPlanModel, DateTime> OrderedTempDic = new Dictionary<ActionPlanModel, DateTime>();

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
                OrderedTempDic.Add(pair.Key, pair.Value);
            }

            // Get last item which is the latest plan.
            ActionPlanModel last = OrderedTempDic.Keys.First();

            if (actionPlans == null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));
            }
            //List<ActionPlanModel> activePoliciesRecords = actionPlans.Select(p => new ActionPlanModel(p.Id, p.Name, p.Description)).ToList();
            return Ok(last);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

}