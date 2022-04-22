﻿using Microsoft.AspNetCore.JsonPatch;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Enums;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using System.Text.Json;
using Microsoft.Extensions.Logging;

namespace Middleware.Common.Repositories
{
    public class InstanceRepository : BaseRepository<InstanceModel>, IInstanceRepository
    {
        public InstanceRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<InstanceRepository> logger) : base(RedisDbIndexEnum.Instance, redisClient, redisGraph, logger)
        {
        }

        
        public async Task<InstanceModel> PatchInstanceAsync(Guid id, InstanceModel patch)
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            InstanceModel currentModel = JsonSerializer.Deserialize<InstanceModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.ImageName))
            {
                currentModel.ImageName = patch.ImageName;
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
            if (patch.ServiceUrl != null && Uri.IsWellFormedUriString(patch.ServiceUrl.ToString(), UriKind.RelativeOrAbsolute))
            {
                currentModel.ServiceUrl = patch.ServiceUrl;
            }
            if (!string.IsNullOrEmpty(patch.ServiceStatus))
            {
                currentModel.ServiceStatus = patch.ServiceStatus;
            }    
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }

    }
}