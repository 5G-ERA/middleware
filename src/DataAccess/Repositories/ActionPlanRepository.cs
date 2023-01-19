﻿using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.DataAccess.Repositories;

public class ActionPlanRepository : BaseRepository<ActionPlanModel>, IActionPlanRepository
{
    /// <summary>
    /// Default constructor
    /// </summary>
    /// <param name="redisClient"></param>
    /// <param name="redisGraph"></param>
    /// <param name="logger"></param>
    public ActionPlanRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<ActionPlanRepository> logger) : base(RedisDbIndexEnum.ActionSequence, redisClient, redisGraph, logger, false)
    {
    }

    /// <summary>
    /// Retrieves actionPlanModels 
    /// </summary>
    /// <returns> List<ActionPlanModel> </returns>
    public async Task<List<ActionPlanModel>> GetActionPlanModelsAsync(Guid robotId)
    {
        List<ActionPlanModel> planModels = await ExecuteLuaQueryAsync("GetPlanByRobotId");

        return planModels;
    }
}