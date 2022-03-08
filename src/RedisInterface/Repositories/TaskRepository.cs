using System.Text.Json;
using Microsoft.AspNetCore.DataProtection.KeyManagement;
using Middleware.Common.Models;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class TaskRepository : BaseRepository<TaskModel>,  ITaskRepository   
    {

        private readonly IConnectionMultiplexer _redisClient;
        

        public TaskRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(6, redisClient, redisGraph)
        {
        }

        public Task<List<TaskModel>> GetTaskAsync()
        {
            throw new NotImplementedException();
        }

        public Task<TaskModel> GetTaskIdAsync(string id)
        {
            throw new NotImplementedException();
        }

        public Task<TaskModel> GetTaskRelationshipsAsync()
        {
            throw new NotImplementedException();
        }
    }
}
