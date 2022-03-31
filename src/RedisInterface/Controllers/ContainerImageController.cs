﻿using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.JsonPatch;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.RedisInterface.Repositories.Abstract;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class ContainerImageController : ControllerBase
    {
        private readonly IContainerImageRepository _containerImageRepository;

        public ContainerImageController(IContainerImageRepository containerImageRepository)
        {
            _containerImageRepository = containerImageRepository ?? throw new ArgumentNullException(nameof(containerImageRepository));
        }

        /// <summary>
        /// Get all the ContainerImageModel entities
        /// </summary>
        /// <returns> the list of ContainerImageModel entities </returns>
        [HttpGet(Name = "ContainerImageGetAll")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<ContainerImageModel>>> GetAllAsync()
        {
            List<ContainerImageModel> models = await _containerImageRepository.GetAllAsync();

            return Ok(models);
        }

        /// <summary>
        /// Get an ContainerImageModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the ContainerImageModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "ContainerImageGetbyId")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            ContainerImageModel model = await _containerImageRepository.GetByIdAsync(id);

            return Ok(model);
        }

        /// <summary>
        /// Add a new ContainerImageModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created ContainerImageModel entity </returns>
        [HttpPost(Name = "ContainerImageAdd")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<ContainerImageModel>> AddAsync([FromBody] ContainerImageModel model)
        {
            await _containerImageRepository.AddAsync(model);
            return Ok(model);
        }

        /// <summary>
        /// Partially update an existing ContainerImageModel entity
        /// </summary>
        /// <param name="patch"></param>
        /// <param name="id"></param>
        /// <returns> the modified ContainerImageModel entity </returns>
        [HttpPatch]
        [Route("{id}", Name = "ContainerImagePatch")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> PatchContainerImageAsync([FromBody] ContainerImageModel patch, [FromRoute] Guid id)
        {
            ContainerImageModel model = await _containerImageRepository.PatchContainerImageAsync(id, patch);
            return Ok(model);
        }


        /// <summary>
        /// Delete a ContainerImageModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "ContainerImageDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _containerImageRepository.DeleteByIdAsync(id);
            return Ok();
        }


        [HttpGet]
        [Route("relation/{name}", Name = "ContainerImageGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            var relations = await _containerImageRepository.GetRelation(id, name);
            return Ok(relations);
        }
    }
}