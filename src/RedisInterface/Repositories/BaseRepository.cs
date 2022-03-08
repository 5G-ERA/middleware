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
        private readonly IRedisGraphClient RedisGraph;


        public BaseRepository(int redisDbIndex, IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph)
        {
            RedisClient = redisClient ?? throw new ArgumentNullException(nameof(redisClient));
            RedisGraph = redisGraph ?? throw new ArgumentNullException(nameof(redisGraph));
            _redisDbIndex = redisDbIndex;
            Db = redisClient.GetDatabase(_redisDbIndex);
        }

        public List<T> ExecuteLuaQuery(string queryName)
        {
            var script = File.ReadAllText(GetScriptPath(queryName));
            
            var prepared = LuaScript.Prepare(script);
            var redisResult = Db.ScriptEvaluate(prepared);

            var models = new List<T>();

            foreach (var pair in redisResult.ToDictionary())
            {
                var val = (string)pair.Value;
                T model = JsonSerializer.Deserialize<T>(val);
                models.Add(model);
            }

            return models;
        }

        private string GetScriptPath(string queryName)
        {
            return Path.Combine(Directory.GetCurrentDirectory(), "LuaQueries", $"{queryName}.lua");
        }
    }
}
