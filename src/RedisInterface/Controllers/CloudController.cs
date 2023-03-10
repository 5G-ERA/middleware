using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Attributes;
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
    public class CloudController : ControllerBase
    {
        private readonly ICloudRepository _cloudRepository;
        private readonly ILogger _logger;

        public CloudController(ICloudRepository cloudRepository, ILogger<CloudController> logger)
        {
            _cloudRepository = cloudRepository ?? throw new ArgumentNullException(nameof(cloudRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));

        }

        /// <summary>
        /// Get all the CloudModel entities
        /// </summary>
        /// <returns> the list of CloudModel entities </returns>
        [HttpGet(Name = "CloudGetAll")]
        [ProducesResponseType(typeof(GetCloudsResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetAllAsync()
        {
            try
            {
                List<CloudModel> models = await _cloudRepository.GetAllAsync();
                if (models.Any() == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Clouds were not found."));
                }
                return Ok(models.ToCloudsResponse());
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Get a CloudModel entity by id 
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the CloudModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "CloudGetById")]
        [ProducesResponseType(typeof(CloudResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                var model = await _cloudRepository.GetByIdAsync(id);
                if (model is null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, $"Cloud with specified id: '{id}' was not found."));
                }
                return Ok(model.ToCloudResponse());
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }


        /// <summary>
        /// Add a new CloudModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created CloudModel entity </returns>
        [HttpPost(Name = "CloudAdd")]
        [ProducesResponseType(typeof(CloudResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> AddAsync([FromBody] CloudRequest model)
        {
            if (model == null)
            {
                return BadRequest("Parameters were not specified.");
            }
            try
            {
                CloudModel cloud = await _cloudRepository.AddAsync(model.ToCloud());
                if (cloud is null)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError,
                        new ApiResponse((int)HttpStatusCode.InternalServerError,
                            "Could not add the cloud to the data store"));
                }
                return Ok(cloud.ToCloudResponse());
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }

        }

        /// <summary>
        /// Partially update an existing CloudModel entity
        /// </summary>
        /// <param name="patch"></param>
        /// <param name="id"></param>
        /// <returns> the modified CloudModel entity </returns>
        [HttpPut]
        [Route("{id}", Name = "CloudPatch")]
        [ProducesResponseType(typeof(CloudResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchCloudAsync([FromMultiSource] UpdateCloudRequest patch)
        {
            try
            {
                var cloud = patch.ToCloud();
                await _cloudRepository.UpdateAsync(cloud);
                var response = cloud.ToCloudResponse();
                return Ok(response);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Delete an CloudModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "CloudDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                var deleted = await _cloudRepository.DeleteByIdAsync(id);
                if (deleted == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "The specified Cloud has not been found."));
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
        [Route("AddRelation", Name = "CloudAddRelation")]
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
                bool isValid = await _cloudRepository.AddRelationAsync(model);
                if (!isValid)
                {
                    return Problem("The relation was not created");
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
        [Route("relation/{name}", Name = "CloudGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _cloudRepository.GetRelation(id, name);
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
        [Route("relations/{firstName}/{secondName}", Name = "CloudGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _cloudRepository.GetRelations(id, relationNames);
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

        [HttpGet]
        [Route("/name/{name}", Name = "CloudGetDataByName")]
        [ProducesResponseType(typeof(CloudResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetCloudResourceDetailsByNameAsync(string name)
        {
            try
            {
                CloudModel cloudResource = await _cloudRepository.GetCloudResourceDetailsByNameAsync(name);
                if (cloudResource is null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));
                }
                return Ok(cloudResource.ToCloudResponse());
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Get the free clouds that have connectivity to the robot
        /// </summary>
        /// <param name="cloudsToCheck"></param>
        /// <returns>list of cloudModel</returns>
        [HttpGet]
        [Route("free", Name = "GetFreeCloudIds")]
        [ProducesResponseType(typeof(GetCloudsResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<List<CloudModel>>> GetFreeCloudsIdsAsync(List<CloudModel> cloudsToCheck)
        {
            try
            {
                if (!cloudsToCheck.Any())
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "No Edge ids were provided"));
                }

                List<CloudModel> edgeFree = await _cloudRepository.GetFreeCloudsIdsAsync(cloudsToCheck);
                if (!edgeFree.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.BadRequest, "There are no busy edges"));
                }

                var response = edgeFree.ToCloudsResponse(); 
                return Ok(response);
            }
            catch (Exception ex)
            {

                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }

        }

        [HttpGet]
        [Route("lessBusy", Name = "GetLessBusyClouds")]
        [ProducesResponseType(typeof(GetCloudsResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetLessBusyCloudsAsync(List<CloudModel> cloudsToCheck)
        {
            try
            {
                if (!cloudsToCheck.Any())
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "No Cloud ids were provided"));
                }

                List<CloudModel> lessBusyCloud = await _cloudRepository.GetLessBusyCloudsAsync(cloudsToCheck);
                if (!lessBusyCloud.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.BadRequest, "There are no busy edges"));
                }

                var response = lessBusyCloud.ToCloudsResponse();
                return Ok(response);
            }
            catch (Exception ex)
            {

                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        ///  Returns the number of containers that are deployed in a cloud entity base on cloud Name. 
        /// </summary>
        /// <param name="name"></param>
        /// <returns>int</returns>
        [HttpGet]
        [Route("{name}/containers/count", Name = "GetNumCloudContainersByName")]
        [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetNumCloudContainersByName(string name)
        {
            try
            {

                int countContainers = await _cloudRepository.GetNumContainersByNameAsync(name);
                return Ok(countContainers);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }

        }

        /// <summary>
        ///  Returns the number of containers that are deployed in a cloud entity base on cloud Id. 
        /// </summary>
        /// <param name="cloudId"></param>
        /// <returns>int</returns>
        [HttpGet]
        [Route("{id}/containers/count", Name = "GetNumCloudContainersById")]
        [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetNumCloudContainersById(Guid cloudId)
        {
            try
            {
                int countContainers = await _cloudRepository.GetNumContainersByIdAsync(cloudId);
                return Ok(countContainers);
            }
            catch (Exception ex)
            {

                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }

        }

        /// <summary>
        ///  Returns bool for status of BusyCloud by Name
        /// </summary>
        /// <param name="name"></param>
        /// <returns>bool</returns>
        [HttpGet]
        [Route("{name}/busy", Name = "IsBusyCloudByName")]
        [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> IsBusyCloudByName(string name)
        {
            try
            {
                if (string.IsNullOrWhiteSpace(name))
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Name was not provided"));

                bool busy = await _cloudRepository.IsBusyCloudByNameAsync(name);
                return Ok(busy);
            }
            catch (Exception ex)
            {

                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }

        }

        /// <summary>
        /// Returns bool for status of BusyCloud by Id
        /// </summary>
        /// <param name="cloudId"></param>
        /// <returns>bool</returns>
        [HttpGet]
        [Route("{id}/busy", Name = "isBusyCloudById")]
        [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> IsBusyCloudById(Guid cloudId)
        {
            try
            {

                bool busy = await _cloudRepository.IsBusyCloudByIdAsync(cloudId);
                return Ok(busy);
            }
            catch (Exception ex)
            {

                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }
    }
}
