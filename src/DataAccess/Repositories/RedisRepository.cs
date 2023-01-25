using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Redis.OM;
using Redis.OM.Searching;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using System.Linq.Expressions;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Exceptions;
using Middleware.Models.ExtensionMethods;

namespace Middleware.DataAccess.Repositories
{
    public class RedisRepository<TModel, TDto> : IRedisRepository<TModel, TDto> where TModel : BaseModel where TDto : Dto
    {
        private const string GraphName = "RESOURCE_PLANNER";
        protected readonly IRedisCollection<TDto> Collection;
        protected IRedisGraphClient RedisGraph { get; }
        private readonly string _entityName;
        /// <summary>
        /// Specifies if the used model should be saved to the graph
        /// </summary>
        protected readonly bool IsWritableToGraph;
        public RedisRepository(RedisConnectionProvider provider, IRedisGraphClient redisGraph, bool isWritableToGraph)
        {
            RedisGraph = redisGraph;
            _entityName = typeof(TModel).GetModelName().ToUpper();
            Collection = provider.RedisCollection<TDto>();
            IsWritableToGraph = isWritableToGraph;
        }
        public async Task<TModel> AddAsync(TModel model)
        {
            TDto dto = ToTDto(model);
            
            var id = await Collection.InsertAsync(dto);
            if (IsWritableToGraph)
            {
                GraphEntityModel newGraphModel = new GraphEntityModel(Guid.Parse(dto.Id), model.Name, _entityName);
                await AddGraphAsync(newGraphModel);
            }
            return ToTModel(dto);
        }
        public async Task<TModel> AddAsync(TModel model, Func<Guid> guidProvider)
        {
            TDto dto = ToTDto(model);
            var id = await Collection.InsertAsync(dto);
            dto.Id = id;
            return ToTModel(dto);
        }
        private async Task DeleteFromGraph(Guid id)
        {
            GraphEntityModel model = new GraphEntityModel(id, _entityName);

            await DeleteGraphModelAsync(model);
        }

        public async Task<bool> DeleteByIdAsync(Guid id)
        {
            var model = await GetByIdAsync(id);
            if (model is null)
                return false;
            TDto dto = ToTDto(model);
            
            await Collection.DeleteAsync(dto);

            if (IsWritableToGraph)
            {
                await DeleteFromGraph(id);
            }
            
            return true;
        }

        public async Task DeleteAsync(TModel model)
        {
            TDto dto = ToTDto(model);
            await Collection.DeleteAsync(dto);
        }

        public async Task<List<TModel>> GetAllAsync()
        {
            IList<TDto> listAsync = await Collection.ToListAsync();
            return listAsync.Select(ToTModel).ToList();
        }

        public async Task<TModel?> GetByIdAsync(Guid id)
        {
            var dto = await Collection.FindByIdAsync(id.ToString());
            if (dto is null)
            {
                return null;
            }
            return ToTModel(dto);
        }

        public async Task<TModel?> FindSingleAsync(Expression<Func<TDto, bool>> predicate)
        {
            var dto = await Collection.SingleOrDefaultAsync(predicate);
            if (dto is null)
            {
                return null;
            }
            return ToTModel(dto);
        }

        public async Task<List<TModel>> FindAsync(Expression<Func<TDto, bool>> predicate)
        {
            IList<TDto> list = await Collection.Where(predicate).ToListAsync();
            return list.Select(ToTModel).ToList();
        }

        public IRedisCollection<TDto> FindQuery(Expression<Func<TDto, bool>> predicate)
        {
            return Collection.Where(predicate);
        }

        public virtual async Task<List<RelationModel>> GetRelation(Guid id, string relationName, RelationDirection direction = RelationDirection.Outgoing)
        {
            if (string.IsNullOrWhiteSpace(relationName))
                throw new ArgumentException($"'{nameof(relationName)}' cannot be null or whitespace.", nameof(relationName));


            relationName = relationName.ToUpper();
            string query = string.Empty;
            List<RelationModel> relationModels = new List<RelationModel>();
            string entityDef = _entityName + " {ID: '" + id + "' }";
            query = RelationDirection.Incoming == direction 
                ? "MATCH (x) MATCH (y) WHERE (x)-[: " + relationName + "]->(y: " + entityDef + " ) RETURN x,y" 
                : "MATCH (x: " + entityDef + "}) MATCH(y) WHERE(x) -[: " + relationName + "]->(y) RETURN x, y";

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
            model.Type = _entityName;

            string query = "CREATE (x: " + model.Type + " {ID: '" + model.Id + "', Type: '" + model.Type +
                           "', Name: '" + model.Name + "'})";
            ResultSet resultSet = await RedisGraph.Query(GraphName, query);

            return (resultSet != null) && (resultSet.Metrics.NodesCreated == 1);
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

            return (resultSet != null) && (resultSet.Metrics.RelationshipsCreated == 1);
        }

        /// <summary>
        /// Removing a model from RedisGraph db
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        public virtual async Task<bool> DeleteGraphModelAsync(GraphEntityModel model)
        {
            model.Type = _entityName;

            string query = "MATCH (e: " + model.Type + " {ID: '" + model.Id + "'}) DELETE e";
            ResultSet resultSet = await RedisGraph.Query(GraphName, query);

            return (resultSet != null) && (resultSet.Metrics.NodesDeleted == 1);
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

            return (resultSet != null) && (resultSet.Metrics.RelationshipsDeleted == 1);
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

        public async Task UpdateAsync(TModel model)
        {
            TDto dto = ToTDto(model);
            await Collection.UpdateAsync(dto);
        }

        protected TDto ToTDto(TModel model)
        {
            return model.ToDto() as TDto ?? throw new MappingException(typeof(TModel), typeof(TDto));
        }

        protected TModel ToTModel(TDto dto)
        {
            return dto.ToModel() as TModel ?? throw new MappingException(typeof(TDto), typeof(TModel));
        }
    }
}
