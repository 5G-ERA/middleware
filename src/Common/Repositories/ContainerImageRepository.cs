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

        public ContainerImageRepository(IInstanceRepository instanceRepository, IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<ContainerImageRepository> logger) : base(RedisDbIndexEnum.Container, redisClient, redisGraph, logger, true)
        {
            _instanceRepository = instanceRepository;
        }

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
            if (!string.IsNullOrEmpty(patch.Timestamp))
            {
                currentModel.Timestamp = patch.Timestamp;
            }
            if (!string.IsNullOrEmpty(patch.Description))
            {
                currentModel.Description = patch.Description;
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
