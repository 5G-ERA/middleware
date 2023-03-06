using Microsoft.IdentityModel.Tokens;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories
{
    public class RedisInstanceRunningRepository : RedisRepository<InstanceRunningModel, InstanceRunningDto>, IInstanceRunningRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public RedisInstanceRunningRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(provider, redisGraph, true, logger)
        {
        }

        /// <summary>
        /// Patching properties for InstaceModel
        /// </summary>
        /// <param name="id"></param>
        /// <param name="patch"></param>
        /// <returns> Patched model </returns>
        public async Task<InstanceRunningModel> PatchInstanceAsync(Guid id, InstanceRunningModel patch)
        {
            InstanceRunningModel? currentModel = await GetByIdAsync(id);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }
            if (!string.IsNullOrEmpty(patch.ServiceType))
            {
                currentModel.ServiceType = patch.ServiceType;
            }
            if (!string.IsNullOrEmpty(patch.ServiceInstanceId.ToString()))
            {
                currentModel.ServiceInstanceId = patch.ServiceInstanceId;
            }
            if (!string.IsNullOrEmpty(patch.ServiceUrl))
            {
                currentModel.ServiceUrl = patch.ServiceUrl;
            }
            if (!string.IsNullOrEmpty(patch.ServiceStatus))
            {
                currentModel.ServiceStatus = patch.ServiceStatus;
            }
            if (!string.IsNullOrEmpty(patch.DeployedTime.ToString()))
            {
                currentModel.DeployedTime = patch.DeployedTime;
            }

            await UpdateAsync(currentModel);
            return currentModel;
        }

    }
}
