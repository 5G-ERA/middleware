using System.Text.Json;
using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class BaseRepository<T> : IBaseRepository<T> where T : BaseModel
    {
        private readonly RedisDbIndexEnum _redisDbIndex;
        protected readonly IConnectionMultiplexer RedisClient;
        protected readonly IDatabase Db;
        protected readonly IRedisGraphClient RedisGraph;

        public BaseRepository(RedisDbIndexEnum redisDbIndex, IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph)
        {
            RedisClient = redisClient ?? throw new ArgumentNullException(nameof(redisClient));
            RedisGraph = redisGraph ?? throw new ArgumentNullException(nameof(redisGraph));
            _redisDbIndex = redisDbIndex;
            Db = redisClient.GetDatabase((int)_redisDbIndex);
        }

        public async Task<T> AddAsync(T model)
        {
            model.Id = Guid.NewGuid();
            await Db.JsonSetAsync(model.Id.ToString(), JsonSerializer.Serialize(model));

            return model;
        }

        public async Task<T> GetByIdAsync(Guid id) 
        { 
            T model = await Db.JsonGetAsync<T>(id.ToString());
            
            return model;
        }

        public async Task<List<T>> GetAllAsync() 
        {
            List<T> models = new List<T>();
            var keys = await GetKeysAsync("GetKeys");

            foreach (string key in keys)
            {
                string value = (string)await Db.JsonGetAsync(key);
                T currentModel = JsonSerializer.Deserialize<T>(value);
                models.Add(currentModel);
            }
            return models;
        }

        public async Task<bool> DeleteByIdAsync(Guid id) 
        {
            int deleted = await Db.JsonDeleteAsync(id.ToString());
            
            return deleted > 0;
        }


        protected async Task<List<T>> ExecuteLuaQueryAsync(string queryName)
        {
            var script = await File.ReadAllTextAsync(GetScriptPath(queryName));
            
            var prepared = LuaScript.Prepare(script);
            var redisResult = await Db.ScriptEvaluateAsync(prepared);

            var models = new List<T>();
            var results = new List<string>();
            if (redisResult.Type == ResultType.MultiBulk)
            {
                results.AddRange(((RedisValue[])redisResult).Select(x => x.ToString()));
            }

            foreach (var result in results)
            {
                T model = JsonSerializer.Deserialize<T>(result);
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
            if (redisResult.Type == ResultType.MultiBulk)
            {
                models.AddRange(((RedisValue[])redisResult).Select(x=>x.ToString()));
            }
            return models;
        }

        private string GetScriptPath(string queryName)
        {
            return Path.Combine(Directory.GetCurrentDirectory(), "LuaQueries", $"{queryName}.lua");
        }
    }
}
