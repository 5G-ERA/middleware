using System.Text.Json;
using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using Middleware.Common.Repositories.Abstract;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.Common.Repositories
{
    public class ContainerImageRepository : BaseRepository<ContainerImageModel>, IContainerImageRepository
    {
        private readonly IInstanceRepository _instanceRepository;

        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="instanceRepository"></param>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public ContainerImageRepository(IInstanceRepository instanceRepository, IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<ContainerImageRepository> logger) : base(RedisDbIndex.Container, redisClient, redisGraph, logger, true)
        {
            _instanceRepository = instanceRepository;
        }

        /// <summary>
        /// Patching properties for ContainerImageModel
        /// </summary>
        /// <param name="id"></param>
        /// <param name="patch"></param>
        /// <returns> Patched model </returns>
        public async Task<ContainerImageModel> PatchContainerImageAsync(Guid id, ContainerImageModel patch)
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            ContainerImageModel currentModel = JsonSerializer.Deserialize<ContainerImageModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }
            if (!string.IsNullOrEmpty(patch.Timestamp.ToString()))
            {
                currentModel.Timestamp = patch.Timestamp;
            }
            if (!string.IsNullOrEmpty(patch.Description))
            {
                currentModel.Description = patch.Description;
            }
            if (!string.IsNullOrEmpty(patch.K8SDeployment))
            {
                currentModel.K8SDeployment = patch.K8SDeployment;
            }
            if (!string.IsNullOrEmpty(patch.K8SService))
            {
                currentModel.K8SService = patch.K8SService;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }

        /// <inheritdoc/>
        public async Task<List<ContainerImageModel>> GetImagesForInstanceAsync(Guid instanceId)
        {

            List<RelationModel> imageRelations = await _instanceRepository.GetRelation(instanceId, "needs");

            List<Guid> actionIds = imageRelations.Select(i => i.PointsTo.Id).ToList();

            List<ContainerImageModel> images = new();
            foreach (var id in actionIds)
            {
                images.Add(await GetByIdAsync(id));
            }

            return images;
        }
    }
}
