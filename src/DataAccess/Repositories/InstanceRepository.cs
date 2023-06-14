using System.Text.Json;
using Microsoft.Extensions.Logging;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.DataAccess.Repositories;

public class InstanceRepository : BaseRepository<InstanceModel>, IInstanceRepository
{
    /// <summary>
    ///     Default constructor
    /// </summary>
    /// <param name="redisClient"></param>
    /// <param name="redisGraph"></param>
    /// <param name="logger"></param>
    public InstanceRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph,
        ILogger<InstanceRepository> logger) : base(RedisDbIndexEnum.Instance, redisClient, redisGraph, logger, true)
    {
    }

    /// <summary>
    ///     Patching properties for InstaceModel
    /// </summary>
    /// <param name="id"></param>
    /// <param name="patch"></param>
    /// <returns> Patched model </returns>
    public async Task<InstanceModel> PatchInstanceAsync(Guid id, InstanceModel patch)
    {
        var model = (string)await Db.JsonGetAsync(id.ToString());
        var currentModel = JsonSerializer.Deserialize<InstanceModel>(model);
        if (currentModel == null) return null;
        if (!string.IsNullOrEmpty(patch.Name)) currentModel.Name = patch.Name;
        if (!string.IsNullOrEmpty(patch.ServiceType)) currentModel.ServiceType = patch.ServiceType;
        if (patch.IsReusable != null) currentModel.IsReusable = patch.IsReusable;
        if (!string.IsNullOrEmpty(patch.DesiredStatus)) currentModel.DesiredStatus = patch.DesiredStatus;
        await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
        return currentModel;
    }

    /// <summary>
    ///     Return alternative instance to the provided instance.
    ///     It will be of the same family and comply with previous ROS versions.
    /// </summary>
    /// <param name="instance"></param>
    /// <returns>InstanceModel</returns>
    public async Task<InstanceModel> FindAlternativeInstance(Guid instanceId)
    {
        var instance = await GetByIdAsync(instanceId);

        var instanceCandidatesFinal = new List<InstanceModel>();
        var PotentialInstanceCandidates = await GetAllAsync();
        foreach (var instanceCandidate in PotentialInstanceCandidates)
        {
            if (instanceCandidate.Id != instance.Id)
            {
                if (instanceCandidate.InstanceFamily == instance.InstanceFamily &&
                    instanceCandidate.RosDistro == instance.RosDistro &&
                    instanceCandidate.RosVersion == instance.RosVersion) instanceCandidatesFinal.Add(instanceCandidate);
            }
        }

        return instanceCandidatesFinal.FirstOrDefault();
    }
}