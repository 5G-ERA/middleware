using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories
{
    public class RedisInstanceRepository : RedisRepository<InstanceModel, InstanceDto>, IInstanceRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public RedisInstanceRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(provider, redisGraph, true, logger)
        {
        }

        /// <summary>
        /// Patching properties for InstaceModel
        /// </summary>
        /// <param name="id"></param>
        /// <param name="patch"></param>
        /// <returns> Patched model </returns>
        public async Task<InstanceModel> PatchInstanceAsync(Guid id, InstanceModel patch)
        {
            InstanceModel? currentModel = await GetByIdAsync(id);
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
            if (patch.IsReusable != null)
            {
                currentModel.IsReusable = patch.IsReusable;
            }
            if (!string.IsNullOrEmpty(patch.DesiredStatus))
            {
                currentModel.DesiredStatus = patch.DesiredStatus;
            }
            if ((patch.ServiceUrl != null) && Uri.IsWellFormedUriString(patch.ServiceUrl.ToString(), UriKind.RelativeOrAbsolute))
            {
                currentModel.ServiceUrl = patch.ServiceUrl;
            }
            if (!string.IsNullOrEmpty(patch.ServiceStatus))
            {
                currentModel.ServiceStatus = patch.ServiceStatus;
            }
            await UpdateAsync(currentModel);
            return currentModel;
        }

        /// <summary>
        /// Return alternative instance to the provided instance.
        /// It will be of the same family and comply with previous ROS versions.
        /// </summary>
        /// <param name="instance"></param>
        /// <returns>InstanceModel</returns>
        public async Task<InstanceModel> FindAlternativeInstance(Guid instanceId)
        {
            InstanceModel? instance = await GetByIdAsync(instanceId);

            List<InstanceModel> instanceCandidatesFinal = new List<InstanceModel>();
            List<InstanceModel> PotentialInstanceCandidates = await GetAllAsync();
            foreach (InstanceModel instanceCandidate in PotentialInstanceCandidates)
            {
                if (instanceCandidate.Id != instance.Id)
                {
                    if ((instanceCandidate.InstanceFamily == instance.InstanceFamily) && (instanceCandidate.ROSDistro == instance.ROSDistro) && (instanceCandidate.RosVersion == instance.RosVersion))
                    {
                        instanceCandidatesFinal.Add(instanceCandidate);
                    }
                }
            }
            return instanceCandidatesFinal.FirstOrDefault();
        }
    }
}
