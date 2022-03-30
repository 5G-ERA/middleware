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
            string model = (string)await Db.JsonGetAsync(id.ToString());
            T newModel = JsonSerializer.Deserialize<T>(model);

            return newModel;
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
                models.AddRange(((RedisValue[])redisResult).Select(x => x.ToString()));
            }
            return models;
        }

        private string GetScriptPath(string queryName)
        {
            return Path.Combine(Directory.GetCurrentDirectory(), "LuaQueries", $"{queryName}.lua");
        }


        public async Task<List<RelationModel>> GetRelation(Guid id, string relationName)
        {
            List<RelationModel> relationModels = new List<RelationModel>();
            relationName = relationName?.ToUpper();
            string query = "MATCH (x: " + _redisDbIndex.ToString().ToUpper() + " {ID: " + id + " }) MATCH (y) WHERE (x)-[: " + relationName + "]->(y) RETURN x,y";
            ResultSet resultSet = await RedisGraph.Query("RESOURCE_PLANNER", query);
            // BB: 24.03.2022
            // We are using the loop with 2 nested loops to retrieve the values from the graph
            // The values are structured in the following way:
            // First result contains the information about the objects that the relation initiates from
            // Second results contains the information about the objects that the relation is pointing to
            // This structure will be universal for the explanation of all the queries on the redis graph
            for (int i = 0; i < resultSet.Results.Count; i++)
            {
                var res = resultSet.Results.ElementAt(i);
                if (i % 2 == 0)
                {
                    foreach (RedisGraphResult node in res.Value)
                    {
                        var relationModel = new RelationModel();
                        relationModel.RelationName = relationName;
                        if (node is Node nd)
                        {
                            SetGraphModelValues(relationModel.InitiatesFrom, nd);
                        }
                        relationModels.Add(relationModel);
                    }
                }
                else
                {
                    foreach (RedisGraphResult node in res.Value)
                    {
                        var idxTmp = res.Value.IndexOf(node);
                        var relationModel = relationModels[idxTmp];
                        if (node is Node nd)
                        {
                            SetGraphModelValues(relationModel.PointsTo, nd);
                        }
                    }
                }
            }
            return relationModels;
        }


        public async Task<List<RelationModel>> GetRelations(Guid id, List<string> relationNames)
        {
            List<RelationModel> relations = new List<RelationModel>();

            foreach (var relationName in relationNames)
            {
                List<RelationModel> currentRelation = await GetRelation(id, relationName);
                relations.AddRange(currentRelation);
            }
            return relations;
        }

        /// <summary>
        /// Sets the values for the <see cref="GraphEntityModel"/> from the specified node
        /// </summary>
        /// <param name="graphEntity"></param>
        /// <param name="nd"></param>
        protected static void SetGraphModelValues(GraphEntityModel graphEntity, Node nd)
        {
            var props = nd.Properties;
            RedisValue id = props["ID"];
            RedisValue type = props["Type"];
            RedisValue name = props["Name"];
            //graphEntity.Name = id.ToString();
            graphEntity.Id = Guid.Parse(id.ToString());
            graphEntity.Type = type.ToString();
            graphEntity.Name = name.ToString();
        }
    }
}
