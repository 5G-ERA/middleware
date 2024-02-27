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

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class ContainerImageController : MiddlewareController
    {
        private readonly IContainerImageRepository _containerImageRepository;
        private readonly ILogger _logger;

        public ContainerImageController(IContainerImageRepository containerImageRepository,
            ILogger<ActionController> logger)
        {
            _containerImageRepository = containerImageRepository ??
                                        throw new ArgumentNullException(nameof(containerImageRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }

        /// <summary>
        /// Get all the ContainerImageModel entities
        /// </summary>
        /// <returns> the list of ContainerImageModel entities </returns>
        [HttpGet(Name = "ContainerImageGetAll")]
        [ProducesResponseType(typeof(GetContainersResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetAllAsync()
        {
            try
            {
                List<ContainerImageModel> models = await _containerImageRepository.GetAllAsync();
                if (models.Any() == false)
                {
                    return ErrorMessageResponse(HttpStatusCode.NotFound, "container",
                        "ContainerImages were not found.");
                }

                var response = models.ToContainersResponse();
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
        /// Get an ContainerImageModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the ContainerImageModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "ContainerImageGetById")]
        [ProducesResponseType(typeof(ContainerResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                ContainerImageModel model = await _containerImageRepository.GetByIdAsync(id);
                if (model == null)
                {
                    return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                        $"ContainerImage with id: {id} was not found.");
                }

                var response = model.ToContainerResponse();
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
        /// Add a new ContainerImageModel entity
        /// </summary>
        /// <param name="request"></param>
        /// <returns> the newly created ContainerImageModel entity </returns>
        [HttpPost(Name = "ContainerImageAdd")]
        [ProducesResponseType(typeof(ContainerResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> AddAsync([FromBody] ContainerRequest request)
        {
            try
            {
                var model = request.ToContainer();
                var existingLoc = await _containerImageRepository.FindSingleAsync(c => c.Name == model.Name);
                if (existingLoc is not null)
                {
                    return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request.Name),
                        "ContainerImage with specified name already exists");
                }

                ContainerImageModel cim = await _containerImageRepository.AddAsync(model);
                if (cim is null)
                {
                    return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request),
                        "Could not add ContainerImage to the data store");
                }

                var response = cim.ToContainerResponse();
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
        /// Partially update an existing ContainerImageModel entity
        /// </summary>
        /// <param name="request"></param>
        /// <returns> the modified ContainerImageModel entity </returns>
        [HttpPut]
        [Route("{id}", Name = "ContainerImagePatch")]
        [ProducesResponseType(typeof(ContainerResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchContainerImageAsync([FromMultiSource] UpdateContainerRequest request)
        {
            try
            {
                var model = request.ToContainer();
                var exists = await _containerImageRepository.GetByIdAsync(model.Id);
                if (exists is null)
                {
                    return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request.Id),
                        $"ContainerImage with id: {request.Id} was not found.");
                }

                await _containerImageRepository.UpdateAsync(model);
                var response = model.ToContainerResponse();
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
        /// Delete a ContainerImageModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "ContainerImageDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                var exists = await _containerImageRepository.GetByIdAsync(id);
                if (exists is null)
                {
                    return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                        $"ContainerImage with id: {id} was not found.");
                }

                await _containerImageRepository.DeleteByIdAsync(id);
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
        /// Creates a new relation between two models
        /// </summary>
        /// <param name="request"></param>
        /// <returns></returns>
        [HttpPost]
        [Route("AddRelation", Name = "ContainerImageAddRelation")]
        [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> AddRelationAsync([FromBody] RelationModel request)
        {
            if (request == null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request),
                    $"Request body was not specified.");
            }

            try
            {
                bool isValid = await _containerImageRepository.AddRelationAsync(request);
                if (!isValid)
                {
                    return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request),
                        $"Relation was not created");
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
        /// Retrieves a single relation by name
        /// </summary>
        /// <param name="id"></param>
        /// <param name="name"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        [HttpGet]
        [Route("relation/{name}", Name = "ContainerImageGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name, string direction = "Outgoing")
        {
            if (string.IsNullOrWhiteSpace(name))
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(name), $"Relation name not specified");
            }

            if (id == Guid.Empty)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), $"ContainerImage id not specified");
            }

            if (Enum.TryParse<RelationDirection>(direction, out var directionEnum) == false)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(direction),
                    $"Wrong Relation direction specified");
            }

            var inputDirection = directionEnum;
            try
            {
                var relations = await _containerImageRepository.GetRelation(id, name, inputDirection);
                if (!relations.Any())
                {
                    return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                        $"Relations with name: {name} were not found.");
                }

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
        /// Retrieves two relations by their names
        /// </summary>
        /// <param name="id"></param>
        /// <param name="firstName"></param>
        /// <param name="secondName"></param>
        /// <returns></returns>
        [HttpGet]
        [Route("relations/{firstName}/{secondName}", Name = "ContainerImageGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _containerImageRepository.GetRelations(id, relationNames);
                if (!relations.Any())
                {
                    return ErrorMessageResponse(HttpStatusCode.NotFound, "relation", $"Relations were not found");
                }

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
        /// Gets the images associated with the specified instance
        /// </summary>
        /// <param name="id">Identifier of the instance</param>
        /// <returns></returns>
        [HttpGet]
        [Route("instance/{id}", Name = "ContainerImageGetForInstance")]
        [ProducesResponseType(typeof(GetContainersResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetImagesForInstanceAsync(Guid id)
        {
            try
            {
                var images = await _containerImageRepository.GetImagesForInstanceAsync(id);
                if (images.Any() == false)
                {
                    return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                        $"Container Image for instance with id: '{id}' has not ben found");
                }

                return Ok(images.ToContainersResponse());
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                    $"An error has occurred: {ex.Message}");
            }
        }
    }
}