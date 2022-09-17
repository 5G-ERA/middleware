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
        public BaseRepository(RedisDbIndexEnum redisDbIndex, IConnectionMultiplexer redisClient,
            IRedisGraphClient redisGraph, ILogger logger, bool isWritableToGraph)
        {
            RedisClient = redisClient ?? throw new ArgumentNullException(nameof(redisClient));
            RedisGraph = redisGraph ?? throw new ArgumentNullException(nameof(redisGraph));
            _redisDbIndex = redisDbIndex;
            Db = redisClient.GetDatabase((int)_redisDbIndex);
            Logger = logger ?? throw new ArgumentNullException(nameof(logger));
            IsWritableToGraph = isWritableToGraph;
        }


        /// <inheritdoc/>
        public virtual async Task<T> AddAsync(T model)
        {
            return await AddAsync(model, () => Guid.NewGuid());
        }

        /// <inheritdoc/>
        public virtual async Task<T> AddAsync(T model, Func<Guid> guidProvider)
        {
            if (guidProvider == null) throw new ArgumentNullException(nameof(guidProvider));

            model.Id = guidProvider.Invoke();
            OperationResult result = await Db.JsonSetAsync(model.Id.ToString(), JsonSerializer.Serialize(model));
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
        public virtual async Task<T> GetByIdAsync(Guid id)
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
        public virtual async Task<List<T>> GetAllAsync()
        {
            List<T> models = new List<T>();
            var keys = await GetKeysAsync("GetKeys");

            foreach (string key in keys)
            {
                string value = (string)await Db.JsonGetAsync(key);
                if (string.IsNullOrWhiteSpace(value))
                    continue;

                T currentModel = JsonSerializer.Deserialize<T>(value);
                models.Add(currentModel);
            }

            return models;
        }

        /// <inheritdoc/>
        public virtual async Task<bool> DeleteByIdAsync(Guid id)
        {
            int deleted = await Db.JsonDeleteAsync(id.ToString());
            bool success = deleted > 0;
            if (success == false)
            {
                return false;
            }

            if (IsWritableToGraph)
            {
                GraphEntityModel model = new GraphEntityModel(id, _redisDbIndex);

                bool isDeleted = await DeleteGraphModelAsync(model);

                return isDeleted;
            }

            return success;
        }

        /// <summary>
        /// Execute the given Lua query against the redis data store
        /// </summary>
        /// <param name="queryName">name of the query to be executed</param>
        /// <returns>List of the objects of hte specified type T</returns>
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

        protected async Task<List<T>> ExecuteLuaQueryWithParamsAsync(string queryName,List<string> parameters)
        {
            //TODO: implement launching lua query with parameters
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

        public virtual async Task<List<string>> GetKeysAsync(string queryName)
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
        public virtual async Task<List<RelationModel>> GetRelation(Guid id, string relationName, RelationDirection direction = RelationDirection.Outgoing)
        {
            string query = "";
            List<RelationModel> relationModels = new List<RelationModel>();

            if (RelationDirection.Incoming == direction)
            {
                
                relationName = relationName?.ToUpper();
                query = "MATCH (x) MATCH (y) WHERE (x)-[: " + relationName + "]->(y: " + _redisDbIndex.ToString().ToUpper() + " {ID: '" + id +
                "' }) RETURN x,y";
            }     
            else
            {
                relationName = relationName?.ToUpper();
                query = "MATCH (x: " + _redisDbIndex.ToString().ToUpper() + " {ID: '" + id +
                               "' }) MATCH (y) WHERE (x)-[: " + relationName + "]->(y) RETURN x,y";
            }
            

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
        public virtual async Task<List<RelationModel>> GetRelations(Guid id, List<string> relationNames)
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
        /// Adding new model into RedisGraph db
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        public virtual async Task<bool> AddGraphAsync(GraphEntityModel model)
        {
            model.Type = _redisDbIndex.ToString().ToUpper();

            string query = "CREATE (x: " + model.Type + " {ID: '" + model.Id + "', Type: '" + model.Type +
                           "', Name: '" + model.Name + "'})";
            ResultSet resultSet = await RedisGraph.Query(GraphName, query);

            return resultSet != null && resultSet.Metrics.NodesCreated == 1;
        }

        /// <summary>
        /// Creating a new relation between two models
        /// </summary>
        /// <param name="relation"></param>
        /// <returns></returns>
        public virtual async Task<bool> AddRelationAsync(RelationModel relation)
        {
            string query = "MATCH (x: " + relation.InitiatesFrom.Type + " {ID: '" + relation.InitiatesFrom.Id +
                           "'}), (c: " + relation.PointsTo.Type + " {ID: '" + relation.PointsTo.Id +
                           "'}) CREATE (x)-[:" + relation.RelationName + "]->(c) ";
            ResultSet resultSet = await RedisGraph.Query(GraphName, query);

            return resultSet != null && resultSet.Metrics.RelationshipsCreated == 1;
        }

        /// <summary>
        /// Removing a model from RedisGraph db
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        public virtual async Task<bool> DeleteGraphModelAsync(GraphEntityModel model)
        {
            model.Type = _redisDbIndex.ToString().ToUpper();

            string query = "MATCH (e: " + model.Type + " {ID: '" + model.Id + "'}) DELETE e";
            ResultSet resultSet = await RedisGraph.Query(GraphName, query);

            return resultSet != null && resultSet.Metrics.NodesDeleted == 1;
        }

        /// <summary>
        /// Removing a relation from the RedisGraph db
        /// </summary>
        /// <param name="relation"></param>
        /// <returns></returns>
        public virtual async Task<bool> DeleteRelationAsync(RelationModel relation)
        {
            string query = "MATCH (x: " + relation.InitiatesFrom.Type + " {ID: '" + relation.InitiatesFrom.Id +
                           "'})-[e: " + relation.RelationName + " ]->(y: " + relation.PointsTo.Type + " {ID: '" +
                           relation.PointsTo.Id + "'}) DELETE e";
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

        public virtual async Task<List<RelationModel>> GetReferencingRelation(Guid id, string relationName)
        {
            //HERE
            throw new NotImplementedException();
        }

        
    }
}