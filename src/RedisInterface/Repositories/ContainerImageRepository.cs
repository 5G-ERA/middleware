using System.Text.Json;
using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using Middleware.RedisInterface.Repositories.Abstract;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class ContainerImageRepository : BaseRepository<ContainerImageModel>, IContainerImageRepository
    {
        private readonly IActionRepository _actionRepository;

        public ContainerImageRepository(IActionRepository actionRepository, IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<ContainerImageRepository> logger) : base(RedisDbIndexEnum.Container, redisClient, redisGraph, logger)
        {
            _actionRepository = actionRepository;
        }

        public async Task<ContainerImageModel> PatchContainerImageAsync(Guid id, ContainerImageModel patch)
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            ContainerImageModel currentModel = JsonSerializer.Deserialize<ContainerImageModel>(model);
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
        public async Task<List<ContainerImageModel>> GetImagesForActionAsync(Guid actionId)
        {

            List<RelationModel> imageRelations = await _actionRepository.GetRelation(actionId, "needs");

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
