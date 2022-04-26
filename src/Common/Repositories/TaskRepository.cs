using System.Text.Json;
using Middleware.Common.Models;
using Middleware.Common.Enums;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using Microsoft.Extensions.Logging;

namespace Middleware.Common.Repositories
{
    public class TaskRepository : BaseRepository<TaskModel>,  ITaskRepository   
    {
        public TaskRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<TaskRepository> logger) : base(RedisDbIndexEnum.Task, redisClient, redisGraph, logger, true)
        {
        }

        public async Task<TaskModel> PatchTaskAsync(Guid id, TaskModel patch) 
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            TaskModel currentModel = JsonSerializer.Deserialize<TaskModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.TaskPriority.ToString()))
            {
                currentModel.TaskPriority = patch.TaskPriority;
            }
            if (!string.IsNullOrEmpty(patch.ActionSequence.ToString()))
            {
                currentModel.ActionSequence = patch.ActionSequence;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;

        }
    }
}
