﻿using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories;

public class RedisInstanceRepository : RedisRepository<InstanceModel, InstanceDto>, IInstanceRepository
{
    /// <summary>
    ///     Default constructor
    /// </summary>
    /// <param name="provider"></param>
    /// <param name="redisGraph"></param>
    /// <param name="logger"></param>
    public RedisInstanceRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) :
        base(provider, redisGraph, true, logger)
    {
    }

    /// <summary>
    ///     Patching properties for InstanceModel
    /// </summary>
    /// <param name="id"></param>
    /// <param name="patch"></param>
    /// <returns> Patched model </returns>
    public async Task<InstanceModel> PatchInstanceAsync(Guid id, InstanceModel patch)
    {
        var currentModel = await GetByIdAsync(id);
        if (currentModel == null) return null;
        if (!string.IsNullOrEmpty(patch.Name)) currentModel.Name = patch.Name;
        if (!string.IsNullOrEmpty(patch.ServiceType)) currentModel.ServiceType = patch.ServiceType;
        if (patch.IsReusable != null) currentModel.IsReusable = patch.IsReusable;
        if (!string.IsNullOrEmpty(patch.DesiredStatus)) currentModel.DesiredStatus = patch.DesiredStatus;
        if (patch.ServiceUrl != null && Uri.IsWellFormedUriString(patch.ServiceUrl, UriKind.RelativeOrAbsolute))
            currentModel.ServiceUrl = patch.ServiceUrl;
        if (patch.RosTopicsPub.Any()) currentModel.RosTopicsPub = patch.RosTopicsPub;
        if (patch.RosTopicsSub.Any()) currentModel.RosTopicsSub = patch.RosTopicsSub;
        if (!string.IsNullOrEmpty(patch.RosVersion.ToString())) currentModel.RosVersion = patch.RosVersion;
        if (!string.IsNullOrEmpty(patch.RosDistro)) currentModel.RosDistro = patch.RosDistro;
        if (patch.Tags != null) currentModel.Tags = patch.Tags;
        if (!string.IsNullOrEmpty(patch.InstanceFamily)) currentModel.InstanceFamily = patch.InstanceFamily;
        if (!string.IsNullOrEmpty(patch.SuccessRate.ToString())) currentModel.SuccessRate = patch.SuccessRate;
        if (patch.ContainerImage != null) currentModel.ContainerImage = patch.ContainerImage;
        //if (!string.IsNullOrEmpty(patch.Ram.ToString())) currentModel.Ram = patch.Ram;
        //if (!string.IsNullOrEmpty(patch.NumberOfCores.ToString()))
        //currentModel.MinimumNumCores = patch.MinimumNumCores;
        //if (!string.IsNullOrEmpty(patch.OnboardedTime.ToString())) currentModel.OnboardedTime = patch.OnboardedTime;
        await UpdateAsync(currentModel);
        return currentModel;
    }

    /// <summary>
    ///     Return alternative instance to the provided instance.
    ///     It will be of the same family and comply with previous ROS versions.
    /// </summary>
    /// <param name="instanceId"></param>
    /// <returns>InstanceModel</returns>
    public async Task<InstanceModel?> FindAlternativeInstance(Guid instanceId)
    {
        var instance = await GetByIdAsync(instanceId);
        if (instance is null) return null;
        var instanceCandidatesFinal = new List<InstanceModel>();
        var potentialInstanceCandidates = await GetAllAsync();
        foreach (var instanceCandidate in potentialInstanceCandidates)
        {
            if (instanceCandidate.Id != instance.Id)
            {
                if (instanceCandidate.InstanceFamily == instance.InstanceFamily &&
                    instanceCandidate.RosDistro == instance.RosDistro &&
                    instanceCandidate.RosVersion == instance.RosVersion)
                    instanceCandidatesFinal.Add(instanceCandidate);
            }
        }

        return instanceCandidatesFinal.FirstOrDefault();
    }
}