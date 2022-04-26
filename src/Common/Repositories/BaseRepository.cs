using System.Text.Json;
using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.Common.Repositories
{
    public class BaseRepository<T> : IBaseRepository<T> where T : BaseModel
    {
        /// <summary>
        /// Name of the graph to be queried for
        /// </summary>
        private const string GraphName = "RESOURCE_PLANNER";
        /// <summary>
        /// Index of the database in the Redis to be used
        /// </summary>
        private readonly RedisDbIndexEnum _redisDbIndex;
        /// <summary>
        /// Redis Client
        /// </summary>
        protected readonly IConnectionMultiplexer RedisClient;
        /// <summary>
        /// Database used to query the Redis
        /// </summary>
        protected readonly IDatabase Db;
        /// <summary>
        /// Redis Graph client
        /// </summary>
        protected readonly IRedisGraphClient RedisGraph;
        /// <summary>
        /// Logger instance
        /// </summary>
        protected readonly ILogger Logger;
        /// <summary>
        /// Specifies if the used model should be saved to the graph
        /// </summary>
        protected readonly bool IsWritableToGraph;
        /// <summary>
        /// Default c-tor
        /// </summary>
        /// <param name="redisDbIndex">Index of the Redis Db to be used</param>
        /// <param name="redisClient">Redis client </param>
        /// <param name="redisGraph">Redis Graph client</param>
        /// <param name="logger">Logger instance</param>
        /// <param name="isWritableToGraph">Should the object be saved on the graph</param>
        /// <exception cref="ArgumentNullException"></exception>
        public BaseRepository(RedisDbIndexEnum redisDbIndex, IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger logger, bool isWritableToGraph)
        {
            RedisClient = redisClient ?? throw new ArgumentNullException(nameof(redisClient));
            RedisGraph = redisGraph ?? throw new ArgumentNullException(nameof(redisGraph));
            _redisDbIndex = redisDbIndex;
            Db = redisClient.GetDatabase((int)_redisDbIndex);
            Logger = logger ?? throw new ArgumentNullException(nameof(logger));
            IsWritableToGraph = isWritableToGraph;
        }


        /// <inheritdoc/>
        public async Task<T> AddAsync(T model)
        {
            return await AddAsync(model, () => Guid.NewGuid());
        }
        /// <inheritdoc/>
        public async Task<T> AddAsync(T model, Func<Guid> guidProvider)
        {
            if (guidProvider == null) throw new ArgumentNullException(nameof(guidProvider));

            model.Id = guidProvider.Invoke();
            var result = await Db.JsonSetAsync(model.Id.ToString(), JsonSerializer.Serialize(model));
            bool retVal = result.IsSuccess;
            if (retVal == false)
            {
                return null;
            }

            if (IsWritableToGraph)
            {
                GraphEntityModel newGraphModel = new GraphEntityModel(model.Id, model.Name, _redisDbIndex);
                retVal &= await AddGraphAsync(newGraphModel);
            }

            return retVal ? model : null;
        }
        /// <inheritdoc/>
        public async Task<T> GetByIdAsync(Guid id)
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            if (string.IsNullOrEmpty(model))
            {
                return default;
            }
            T newModel = JsonSerializer.Deserialize<T>(model);

            return newModel;
        }
        /// <inheritdoc/>
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
        /// <inheritdoc/>
        public async Task<bool> DeleteByIdAsync(Guid id)
        {
            int deleted = await Db.JsonDeleteAsync(id.ToString());

            return deleted > 0;
        }
        /// <summary>
        /// Execute the given Lua query against the redis data store
        /// </summary>
        /// <param name="queryName"></param>
        /// <returns></returns>
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
        /// <inheritdoc/>
        public async Task<List<RelationModel>> GetRelation(Guid id, string relationName)
        {
            List<RelationModel> relationModels = new List<RelationModel>();
            relationName = relationName?.ToUpper();
            string query = "MATCH (x: " + _redisDbIndex.ToString().ToUpper() + " {ID: '" + id + "' }) MATCH (y) WHERE (x)-[: " + relationName + "]->(y) RETURN x,y";
            ResultSet resultSet = await RedisGraph.Query(GraphName, query);
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
                        var relationModel = new RelationModel
                        {
                            RelationName = relationName
                        };
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

        /// <inheritdoc/>
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


        public async Task<bool> AddGraphAsync(GraphEntityModel model)
        {
            model.Type = _redisDbIndex.ToString().ToUpper();

            string query = "CREATE (x: " + model.Type + " {ID: '" + model.Id + "', Type: '" + model.Type + "', Name: '" + model.Name + "'})";
            ResultSet resultSet = await RedisGraph.Query(GraphName, query);

            return resultSet != null && resultSet.Metrics.NodesCreated == 1;
        }


        public async Task<bool> AddRelationAsync(GraphEntityModel model, string relationName)
        {
            model.Type = _redisDbIndex.ToString().ToUpper();
            relationName = relationName?.ToUpper();

            string query = "MATCH (x: " + model.Type + " {ID: '" + model.Id + "'}), (c: " + model.Type + " {ID: '" + model.Id + "'}) CREATE (x)-[:" + relationName + "]->(c) ";
            ResultSet resultSet = await RedisGraph.Query(GraphName, query);

            return resultSet != null && resultSet.Metrics.RelationshipsCreated == 1;
        }


        public async Task<bool> DeleteGraphModelAsync(GraphEntityModel model)
        {
            model.Type = _redisDbIndex.ToString().ToUpper();

            string query = "MATCH (e: " + model.Type + " {ID: '" + model.Id + "', Name: '" + model.Name + "'}) DELETE e";
            ResultSet resultSet = await RedisGraph.Query(GraphName, query);

            return resultSet != null && resultSet.Metrics.NodesDeleted == 1;
        }


        public async Task<bool> DeleteRelationAsync(GraphEntityModel model, string relationName)
        {
            model.Type = _redisDbIndex.ToString().ToUpper();
            relationName = relationName?.ToUpper();

            string query = "MATCH (x: " + model.Type + " {ID: '" + model.Id + "'})-[e: " + relationName + " ]->(y: " + model.Type + " {ID: '" + model.Id + "'}) DELETE e";
            ResultSet resultSet = await RedisGraph.Query(GraphName, query);

            return resultSet != null && resultSet.Metrics.RelationshipsDeleted == 1;
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
