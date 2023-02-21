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


namespace Middleware.DataAccess.Repositories.Redis
{
    public class RedisTaskRepository : RedisRepository<TaskModel, TaskDto>, ITaskRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>

        public RedisTaskRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(provider, redisGraph, true, logger)
        {
        }

        /// <summary>
        /// Patching properties for TaskModel
        /// </summary>
        /// <param name="id"></param>
        /// <param name="patch"></param>
        /// <returns> Patched model </returns>
        public async Task<TaskModel> PatchTaskAsync(Guid id, TaskModel patch)
        {
            TaskModel? currentModel = await GetByIdAsync(id);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }
            if (patch.ReplanActionPlannerLocked != null)
            {
                currentModel.ReplanActionPlannerLocked = patch.ReplanActionPlannerLocked;
            }
            if (patch.ResourceLock != null)
            {
                currentModel.ResourceLock = patch.ResourceLock;
            }
            if (!string.IsNullOrEmpty(patch.TaskPriority.ToString()))
            {
                currentModel.TaskPriority = patch.TaskPriority;
            }
            if (patch.FullReplan != null)
            {
                currentModel.FullReplan = patch.FullReplan;
            }
            if (patch.PartialRePlan != null)
            {
                currentModel.PartialRePlan = patch.PartialRePlan;
            }
            if (patch.DeterministicTask != null)
            {
                currentModel.DeterministicTask = patch.DeterministicTask;
            }
            if (patch.MarkovianProcess != null)
            {
                currentModel.MarkovianProcess = patch.MarkovianProcess;
            }
            if (patch.ActionSequence != null)
            {
                currentModel.ActionSequence = patch.ActionSequence;
            }
            if (patch.Tags != null)
            {
                currentModel.Tags = patch.Tags;
            }
            await UpdateAsync(currentModel);
            return currentModel;

        }

    }
}
