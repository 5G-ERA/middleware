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

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class RobotController : MiddlewareController
{
    private readonly ILogger _logger;
    private readonly IRobotRepository _robotRepository;

    public RobotController(IRobotRepository robotRepository, ILogger<RobotController> logger)
    {
        _robotRepository = robotRepository ?? throw new ArgumentNullException(nameof(robotRepository));
        _logger = logger ?? throw new ArgumentNullException(nameof(logger));
    }

    /// <summary>
    ///     Get all the RobotModel entities
    /// </summary>
    /// <returns> the list of RobotModel entities </returns>
    [HttpGet(Name = "RobotGetAll")]
    [ProducesResponseType(typeof(GetRobotsResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetAllAsync()
    {
        try
        {
            var models = await _robotRepository.GetAllAsync();
            if (models.Any() == false)
                return ErrorMessageResponse(HttpStatusCode.NotFound, "robot", "No Robots were found.");

            var response = models.ToRobotsResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Get a RobotModel entity by id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> the RobotModel entity for the specified id </returns>
    [HttpGet]
    [Route("{id}", Name = "RobotGetById")]
    [ProducesResponseType(typeof(RobotResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetByIdAsync(Guid id)
    {
        try
        {
            var model = await _robotRepository.GetByIdAsync(id);
            if (model == null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                    $"Robot with specified id: {id} was not found.");

            var response = model.ToRobotResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Add a new RobotModel entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the newly created RobotModel entity </returns>
    [HttpPost(Name = "RobotAdd")]
    [ProducesResponseType(typeof(RobotResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddAsync([FromBody] RobotRequest request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Request body was not specified.");

        try
        {
            var model = request.ToRobot();
            var existingRobot = await _robotRepository.FindSingleAsync(r => r.Name == model.Name);
            if (existingRobot is not null)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request.Name),
                    $"Robot with specified name: {request.Name} already exists.");
            }

            model.OnboardedTime = model.LastUpdatedTime;
            var robot = await _robotRepository.AddAsync(model);
            if (robot is null)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request),
                    $"Could not add the robot to the data store.");
            }

            var response = robot.ToRobotResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Partially update an existing InstanceModel entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the modified InstanceModel entity </returns>
    [HttpPut]
    [Route("{id}", Name = "RobotPatch")]
    [ProducesResponseType(typeof(RobotResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> UpdateRobotAsync([FromMultiSource] UpdateRobotRequest request)
    {
        try
        {
            var robot = request.ToRobot();
            var exists = await _robotRepository.GetByIdAsync(robot.Id);
            if (exists is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request.Id),
                    $"Robot with specified id: {request.Id} was not found.");

            robot.OnboardedTime = exists.OnboardedTime;
            await _robotRepository.UpdateAsync(robot);
            var response = robot.ToRobotResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Delete a RobotModel entity for the given id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> no return </returns>
    [HttpDelete]
    [Route("{id}", Name = "RobotDelete")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteByIdAsync(Guid id)
    {
        try
        {
            var exists = await _robotRepository.GetByIdAsync(id);
            if (exists is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                    $"Robot with specified id: {id} was not found.");

            await _robotRepository.DeleteByIdAsync(id);
            return Ok();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Creates a new relation between two models
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("AddRelation", Name = "RobotAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddRelationAsync([FromBody] RelationModel request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"Request body was not specified.");

        try
        {
            var isValid = await _robotRepository.AddRelationAsync(request);
            if (!isValid)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request),
                    $"The relation was not created.");
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }

        return Ok(request);
    }

    /// <summary>
    ///     Deletes a new relation between two models
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("DeleteRelation", Name = "RobotDeleteRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteRelationAsync([FromBody] RelationModel request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"Request body was not specified.");

        try
        {
            var isValid = await _robotRepository.DeleteRelationAsync(request);
            if (!isValid)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, "relation", $"The relation was not created.");
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }

        return Ok(request);
    }


    /// <summary>
    ///     Retrieves a single relation by name
    /// </summary>
    /// <param name="id"></param>
    /// <param name="name"></param>
    /// <param name="direction"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("relation/{name}", Name = "RobotGetRelationByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationAsync(Guid id, string name, string direction = "Outgoing")
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(name), $"Relation name not specified.");
        }

        if (id == Guid.Empty)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), $"Robot ID not specified.");
        }

        if (Enum.TryParse<RelationDirection>(direction, out var directionEnum) == false)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(direction),
                $"Wrong Relation direction specified.");
        }

        var inputDirection = directionEnum;
        try
        {
            var relations = await _robotRepository.GetRelation(id, name, inputDirection);
            if (!relations.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, "relation", $"The relation was not found.");

            return Ok(relations);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Retrieves two relations by their names
    /// </summary>
    /// <param name="id"></param>
    /// <param name="firstName"></param>
    /// <param name="secondName"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("relations/{firstName}/{secondName}", Name = "RobotGetRelationsByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
    {
        try
        {
            var relationNames = new List<string> { firstName, secondName };
            var relations = await _robotRepository.GetRelations(id, relationNames);
            if (!relations.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, "relations", $"The relations were not found.");

            return Ok(relations);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }


    [HttpGet]
    [Route("{id}/edges/connected", Name = "RobotGetConnectedEdgesIds")]
    [ProducesResponseType(typeof(GetEdgesResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetConnectedEdgesIdsAsync(Guid id)
    {
        try
        {
            if (id == Guid.Empty)
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), $"Robot ID not specified.");

            var exists = await _robotRepository.GetByIdAsync(id);
            if (exists is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                    $"Robot with specified id: {id} was not found.");

            var connectedEdges = await _robotRepository.GetConnectedEdgesIdsAsync(id);
            if (!connectedEdges.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, "edges", $"No connected Edges have been found.");

            var response = connectedEdges.ToEdgesResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Return the connected clouds to the robot
    /// </summary>
    /// <param name="id"></param>
    /// <returns>returns a list of cloudModels that have connectivity to the robot</returns>
    [HttpGet]
    [Route("{id}/clouds/connected", Name = "RobotGetConnectedCloudsIds")]
    [ProducesResponseType(typeof(GetCloudsResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetConnectedCloudsIdsAsync(Guid id)
    {
        try
        {
            if (id == Guid.Empty)
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), $"Robot ID not specified.");

            var exists = await _robotRepository.GetByIdAsync(id);
            if (exists is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                    $"Robot with specified id: {id} was not found.");

            var connectedClouds = await _robotRepository.GetConnectedCloudsIdsAsync(id);
            if (!connectedClouds.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, "clouds", $"No connected clouds have been found.");

            var response = connectedClouds.ToCloudsResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    [HttpPut]
    [Route("{robotId}/topic", Name = "RobotChangeTopicStatus")]
    [ProducesResponseType(typeof(UpdateTopicResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> EnableTopicAsync([FromMultiSource] UpdateRobotTopicRequest request)
    {
        try
        {
            var robot = await _robotRepository.GetByIdAsync(request.Id);
            if (robot is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request.Id), $"Robot with id {request.Id} was not found.");
                
            var topicModel = robot.GetTopicModelFromRobot(request.TopicName);
            if (topicModel is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, "topic", $"Topic for specified Robot was not found.");
                
            topicModel.Enabled = request.Enabled;
            await _robotRepository.UpdateAsync(robot);

            var response = new UpdateTopicResponse
            {
                RobotId = request.Id,
                TopicName = topicModel.Name,
                TopicType = topicModel.Type,
                TopicDescription = topicModel.Description,
                TopicEnabled = topicModel.Enabled
            };
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }
}