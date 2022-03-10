using System.Text.Json;
using Microsoft.AspNetCore.DataProtection.KeyManagement;
using Middleware.Common.Models;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using NReJSON;

namespace Middleware.RedisInterface.Repositories
{
    public class TaskRepository : BaseRepository<TaskModel>,  ITaskRepository   
    {

        
        

        public TaskRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(6, redisClient, redisGraph)
        {
        }

        public async Task<List<TaskModel>> GetAllTasksAsync(Guid id)
        {
            List<TaskModel> tasks = new List<TaskModel>();
            var keys = await GetKeysAsync("GetKeys");
            //

            foreach (string key in keys) 
            {
                string value = (string)await Db.JsonGetAsync(key);
                TaskModel currentTask = JsonSerializer.Deserialize<TaskModel>(value);
                tasks.Add(currentTask);
            }
            return tasks;

            //List<TaskModel> currentTasks = JsonSerializer.Deserialize<List<TaskModel>>(objects);

            //foreach (char val in id) 
            //{
            //    string task = (string)await Db.JsonGetAsync(id.ToString());

            //    if (string.IsNullOrEmpty(task))
            //        return tasks;
            //    TaskModel currentTask = JsonSerializer.Deserialize<TaskModel>(val);
            //    tasks.Add(currentTask);
            //}
            
        }

        public Task<TaskModel> GetTaskByIdAsync(Guid id)
        {
            throw new NotImplementedException();
        }

        public Task<TaskModel> GetTaskRelationshipsAsync()
        {
            throw new NotImplementedException();
        }
    }
}
