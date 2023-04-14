using System.Collections.Generic;
using System.Text.Json;
using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Enums;
using NReJSON;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using ILogger = Serilog.ILogger;


namespace Middleware.DataAccess.Repositories
{
    public class RedisHistoricalActionPlanRepository : RedisRepository<HistoricalActionPlanModel, HistoricalActionPlanDto>, IHistoricalActionPlanRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public RedisHistoricalActionPlanRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph,
            ILogger logger) : base(provider, redisGraph, true, logger)
        {
        }

        /// <summary>
        /// Retrieves all actionPlanModels associated with an specific robot Id.
        /// </summary>
        /// <returns> List of ActionPlanModel </returns>
        public async Task<List<HistoricalActionPlanModel>> GetRobotActionPlans(Guid robotId)
        {
            var guidStr = robotId.ToString();
            var actionPlans = await FindQuery(dto => dto.RobotId == guidStr).ToListAsync();
            var planModels = actionPlans.Select(ToTModel).ToList();
            return planModels;
        }

        /// <summary>
        /// Retrieves all actionPlanModels associated with an specific robot Id that have a replan set to true.
        /// </summary>
        /// <returns> List of HistoricalActionPlanModel </returns>
        public async Task<List<HistoricalActionPlanModel>> GetRobotReplanActionPlans(Guid robotId)
        {
            var guidStr = robotId.ToString();
            var actionPlans = await FindQuery(dto => dto.RobotId == guidStr && dto.IsReplan == true).ToListAsync();
            var planModels = actionPlans.Select(ToTModel).ToList();
            return planModels;
        }
    }
}