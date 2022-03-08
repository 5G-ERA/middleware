using Middleware.Common.Models;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class TaskRepository : ITaskRepository   
    {

        private readonly IConnectionMultiplexer _redisClient;
        private readonly IRedisGraphClient _redisGraph;

        public TaskRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph)
        {
            _redisClient = redisClient ?? throw new ArgumentNullException(nameof(redisClient));
            _redisGraph = redisGraph ?? throw new ArgumentNullException(nameof(redisGraph));
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
