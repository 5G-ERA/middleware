using System.Text.Json;
using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class TaskRepository : BaseRepository<TaskModel>,  ITaskRepository   
    {
        public TaskRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<TaskRepository> logger) : base(RedisDbIndexEnum.Task, redisClient, redisGraph, logger)
        {
        }

        public async Task<TaskModel> PatchTaskAsync(Guid id, TaskModel patch) 
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            TaskModel currentModel = JsonSerializer.Deserialize<TaskModel>(model);
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
