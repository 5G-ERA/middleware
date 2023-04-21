using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using ILogger = Serilog.ILogger;

namespace Middleware.DataAccess.Repositories
{
    public class RedisContainerImageRepository : RedisRepository<ContainerImageModel, ContainerImageDto>, IContainerImageRepository
    {
        private readonly IInstanceRepository _instanceRepository;

        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="instanceRepository"></param>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>

        public RedisContainerImageRepository(IInstanceRepository instanceRepository, IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(provider, redisGraph, true, logger)
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
            ContainerImageModel? currentModel = await GetByIdAsync(id);
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
            await UpdateAsync(currentModel);
            return currentModel;
        }

        /// <inheritdoc/>
        public async Task<List<ContainerImageModel>> GetImagesForInstanceAsync(Guid instanceId)
        {
            //TODO: extract this method to Container Image Service
            List<RelationModel> imageRelations = await _instanceRepository.GetRelation(instanceId, "needs");

            List<Guid> containerIds = imageRelations.Select(i => i.PointsTo.Id).ToList();
            List<ContainerImageModel> images = new();
            foreach (var item in containerIds)
            {
                var list = await GetByIdAsync(item);
                if (list is null )
                    continue;
                images.Add(list);
            }

             //= await FindAsync(c => containerIds.Contains(c.Id));
            //List<ContainerImageModel> images = new();
            //foreach (var id in actionIds)
            //{
            //  images.Add(await GetByIdAsync(id));
            //}
            return images;
        }
    }
}
