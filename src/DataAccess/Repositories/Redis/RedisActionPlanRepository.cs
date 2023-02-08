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
    public class RedisActionPlanRepository : RedisRepository<ActionPlanModel, ActionPlanDto>, IActionPlanRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>

        public RedisActionPlanRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(provider, redisGraph, true, logger)
        {
        }

        /// <summary>
        /// Retrieves all actionPlanModels associated with an specific robot Id.
        /// </summary>
        /// <returns> List<ActionPlanModel> </returns>
        public async Task<List<ActionPlanModel>> GetActionPlanModelsAsync(Guid robotId)
        {
            var ActionPlans = FindQuery(Dto => Dto.RobotId == robotId.ToString()).ToList().Select(x => ToTModel(x)).ToList();
            return ActionPlans;
        }


    }
}
