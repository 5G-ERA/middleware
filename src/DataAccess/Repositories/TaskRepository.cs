﻿using System.Text.Json;
using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using Middleware.DataAccess.Repositories.Abstract;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.DataAccess.Repositories
{
    public class TaskRepository : BaseRepository<TaskModel>, ITaskRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public TaskRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<TaskRepository> logger) : base(RedisDbIndexEnum.Task, redisClient, redisGraph, logger, true)
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
            string model = (string)await Db.JsonGetAsync(id.ToString());
            TaskModel currentModel = JsonSerializer.Deserialize<TaskModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }
            if (!string.IsNullOrEmpty(patch.TaskPriority.ToString()))
            {
                currentModel.TaskPriority = patch.TaskPriority;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;

        }
    }
}
