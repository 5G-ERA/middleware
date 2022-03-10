using System.Text.Json;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class BaseRepository<T> : IBaseRepository<T> where T : class
    {
        private readonly int _redisDbIndex;
        protected readonly IConnectionMultiplexer RedisClient;
        protected readonly IDatabase Db;
        protected readonly IRedisGraphClient RedisGraph;


        public BaseRepository(int redisDbIndex, IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph)
        {
            RedisClient = redisClient ?? throw new ArgumentNullException(nameof(redisClient));
            RedisGraph = redisGraph ?? throw new ArgumentNullException(nameof(redisGraph));
            _redisDbIndex = redisDbIndex;
            Db = redisClient.GetDatabase(_redisDbIndex);
        }

        public async Task<List<T>> ExecuteLuaQueryAsync(string queryName)
        {
            var script = await File.ReadAllTextAsync(GetScriptPath(queryName));
            
            var prepared = LuaScript.Prepare(script);
            var redisResult = await Db.ScriptEvaluateAsync(prepared);

            var models = new List<T>();

            foreach (var pair in redisResult.ToDictionary())
            {
                var val = (string)pair.Value;
                T model = JsonSerializer.Deserialize<T>(val);
                models.Add(model);
            }

            return models;
        }

        public async Task<List<string>> GetKeysAsync(string queryName)
        {
            var script = await File.ReadAllTextAsync(GetScriptPath(queryName));

            var prepared = LuaScript.Prepare(script);
            var redisResult = await Db.ScriptEvaluateAsync(prepared);

            var models = new List<string>();

            foreach (var pair in redisResult.ToDictionary())
            {
               models.Add((string)pair.Value);
            }

            return models;
        }

        private string GetScriptPath(string queryName)
        {
            return Path.Combine(Directory.GetCurrentDirectory(), "LuaQueries", $"{queryName}.lua");
        }
    }
}
