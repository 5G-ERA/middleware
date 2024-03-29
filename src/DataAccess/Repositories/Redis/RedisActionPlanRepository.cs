﻿using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using ILogger = Serilog.ILogger;


namespace Middleware.DataAccess.Repositories
{
    public class RedisActionPlanRepository : RedisRepository<ActionPlanModel, ActionPlanDto>, IActionPlanRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public RedisActionPlanRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph,
            ILogger logger) : base(provider, redisGraph, true, logger)
        {
        }

        /// <summary>
        /// Retrieves all actionPlanModels associated with an specific robot Id.
        /// </summary>
        /// <returns> List of ActionPlanModel </returns>
        public async Task<List<ActionPlanModel>> GetRobotActionPlans(Guid robotId)
        {
            var guidStr = robotId.ToString();
            var actionPlans = await FindQuery(dto => dto.RobotId == guidStr).ToListAsync();
            var planModels = actionPlans.Select(ToTModel).ToList();
            return planModels;
        }
    }
}