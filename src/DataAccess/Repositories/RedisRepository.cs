using System.Linq.Expressions;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Exceptions;
using Middleware.Models.ExtensionMethods;
using Redis.OM;
using Redis.OM.Contracts;
using Redis.OM.Searching;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories;

public class RedisRepository<TModel, TDto> : IRedisRepository<TModel, TDto> where TModel : BaseModel where TDto : Dto
{
    private const string GraphName = "RESOURCE_PLANNER";
    private readonly string _entityName;
    protected readonly IRedisCollection<TDto> Collection;

    /// <summary>
    ///     Specifies if the used model should be saved to the graph
    /// </summary>
    protected readonly bool IsWritableToGraph;

    protected readonly ILogger Logger;

    protected IRedisGraphClient RedisGraph { get; }

    public RedisRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, bool isWritableToGraph,
        ILogger logger)
    {
        RedisGraph = redisGraph;
        _entityName = typeof(TModel).GetModelName().ToUpper();
        Collection = provider.RedisCollection<TDto>();
        IsWritableToGraph = isWritableToGraph;
        Logger = logger;
    }

    /// <summary>
    ///     Add to redis a model and try adding also to the graph.
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    public async Task<TModel?> AddAsync(TModel model)
    {
        var dto = ToTDto(model);

        var id = await Collection.InsertAsync(dto);
        if (IsWritableToGraph)
        {
            var newGraphModel = new GraphEntityModel(Guid.Parse(dto.Id), model.Name, _entityName);
            await AddGraphAsync(newGraphModel);
        }

        return ToTModel(dto);
    }

    /// <summary>
    ///     Add a model with specified identifier of type Guid.
    /// </summary>
    /// <param name="model"></param>
    /// <param name="guidProvider"></param>
    /// <returns></returns>
    public async Task<TModel> AddAsync(TModel model, Func<Guid> guidProvider)
    {
        var dto = ToTDto(model);
        var id = await Collection.InsertAsync(dto);
        if (string.IsNullOrEmpty(id)) return null!;

        if (IsWritableToGraph)
        {
            var graphModel = new GraphEntityModel(model.Id, model.Name, _entityName);
            await AddGraphAsync(graphModel);
        }

        return ToTModel(dto);
    }

    public async Task<bool> DeleteByIdAsync(Guid id)
    {
        var model = await GetByIdAsync(id);
        if (model is null)
            return false;
        var dto = ToTDto(model);

        await Collection.DeleteAsync(dto);

        if (IsWritableToGraph) await DeleteFromGraph(id);

        return true;
    }

    public async Task DeleteAsync(TModel model)
    {
        var dto = ToTDto(model);
        await Collection.DeleteAsync(dto);
    }

    public async Task<List<TModel>> GetAllAsync()
    {
        var listAsync = await Collection.ToListAsync();
        return listAsync.Select(ToTModel).ToList();
    }

    public async Task<TModel?> GetByIdAsync(Guid id)
    {
        var dto = await Collection.FindByIdAsync(id.ToString());
        if (dto is null) return null;
        return ToTModel(dto);
    }

    /// <summary>
    ///     Find a single instance of a model
    /// </summary>
    /// <param name="query"></param>
    /// <returns></returns>
    public async Task<TModel?> FindSingleAsync(Expression<Func<TDto, bool>> query)
    {
        var dto = await Collection.SingleOrDefaultAsync(query);
        if (dto is null) return null;
        return ToTModel(dto);
    }

    /// <summary>
    ///     Return all the models that match the query
    /// </summary>
    /// <param name="predicate"></param>
    /// <returns></returns>
    public async Task<List<TModel>> FindAsync(Expression<Func<TDto, bool>> predicate)
    {
        var list = await Collection.Where(predicate).ToListAsync();
        return list.Select(ToTModel).ToList();
    }

    /// <summary>
    ///     Returns a query that is not yet executed.
    /// </summary>
    /// <param name="predicate"></param>
    /// <returns></returns>
    public IRedisCollection<TDto> FindQuery(Expression<Func<TDto, bool>> predicate)
    {
        return Collection.Where(predicate);
    }

    public virtual async Task<List<RelationModel>> GetRelation(Guid id, string relationName,
        RelationDirection direction = RelationDirection.Outgoing)
    {
        if (string.IsNullOrWhiteSpace(relationName))
        {
            throw new ArgumentException($"'{nameof(relationName)}' cannot be null or whitespace.",
                nameof(relationName));
        }


        relationName = relationName.ToUpper();
        var relationModels = new List<RelationModel>();
        var entityDef = _entityName + " {ID: '" + id + "' }";
        var query = RelationDirection.Incoming == direction
            ? "MATCH (x) MATCH (y) WHERE (x)-[: " + relationName + "]->(y: " + entityDef + " ) RETURN x,y"
            : "MATCH (x: " + entityDef + ") MATCH(y) WHERE(x) -[: " + relationName + "]->(y) RETURN x, y";

        var resultSet = await RedisGraph.Query(GraphName, query);
        // BB: 24.03.2022
        // We are using the loop with 2 nested loops to retrieve the values from the graph
        // The values are structured in the following way:
        // First result contains the information about the objects that the relation initiates from
        // Second results contains the information about the objects that the relation is pointing to
        // This structure will be universal for the explanation of all the queries on the redis graph
        for (var i = 0; i < resultSet.Results.Count; i++)
        {
            var res = resultSet.Results.ElementAt(i);
            if (i % 2 == 0)
            {
                foreach (var node in res.Value)
                {
                    var relationModel = new RelationModel
                    {
                        RelationName = relationName
                    };
                    if (node is Node nd) SetGraphModelValues(relationModel.InitiatesFrom, nd);

                    relationModels.Add(relationModel);
                }
            }
            else
            {
                foreach (var node in res.Value)
                {
                    var idxTmp = res.Value.IndexOf(node);
                    var relationModel = relationModels[idxTmp];
                    if (node is Node nd) SetGraphModelValues(relationModel.PointsTo, nd);
                }
            }
        }

        return relationModels;
    }

    /// <inheritdoc />
    public virtual async Task<List<RelationModel>> GetRelations(Guid id, List<string> relationNames)
    {
        var relations = new List<RelationModel>();

        foreach (var relationName in relationNames)
        {
            var currentRelation = await GetRelation(id, relationName);
            relations.AddRange(currentRelation);
        }

        return relations;
    }

    /// <summary>
    ///     Return all RelationModels to recreate the graph.
    /// </summary>
    /// <returns></returns>
    public virtual async Task<Dictionary<string, List<RedisGraphResult>>> GetAllRelations()
    {
        var query = "MATCH (n) OPTIONAL MATCH (n)-[r]->(m) RETURN n, type(r) as r, m";
        var resultSet = await RedisGraph.Query(GraphName, query);

        return resultSet?.Results;
    }

    /// <summary>
    ///     Adding new model into RedisGraph db
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    public virtual async Task<bool> AddGraphAsync(GraphEntityModel model)
    {
        model.Type = _entityName;
        if (await ObjectExistsOnGraph(model)) return true;
        var query = "CREATE (x: " + model.Type + " {ID: '" + model.Id + "', Type: '" + model.Type +
                    "', Name: '" + model.Name + "'})";
        var resultSet = await RedisGraph.Query(GraphName, query);

        return resultSet != null && resultSet.Metrics.NodesCreated == 1;
    }

    /// <summary>
    ///     Creating a new relation between two models
    /// </summary>
    /// <param name="relation"></param>
    /// <returns></returns>
    public virtual async Task<bool> AddRelationAsync(RelationModel relation)
    {
        var query = "MATCH (x: " + relation.InitiatesFrom.Type + " {ID: '" + relation.InitiatesFrom.Id +
                    "'}), (c: " + relation.PointsTo.Type + " {ID: '" + relation.PointsTo.Id +
                    "'}) CREATE (x)-[:" + relation.RelationName + "]->(c) ";
        var resultSet = await RedisGraph.Query(GraphName, query);

        return resultSet != null && resultSet.Metrics.RelationshipsCreated >= 1;
    }

    /// <summary>
    ///     Removing a model from RedisGraph db
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    public virtual async Task<bool> DeleteGraphModelAsync(GraphEntityModel model)
    {
        model.Type = _entityName;

        var query = "MATCH (e: " + model.Type + " {ID: '" + model.Id + "'}) DELETE e";
        var resultSet = await RedisGraph.Query(GraphName, query);

        return resultSet != null && resultSet.Metrics.NodesDeleted == 1;
    }

    /// <summary>
    ///     Removing a relation from the RedisGraph db
    /// </summary>
    /// <param name="relation"></param>
    /// <returns></returns>
    public virtual async Task<bool> DeleteRelationAsync(RelationModel relation)
    {
        var query = "MATCH (x: " + relation.InitiatesFrom.Type + " {ID: '" + relation.InitiatesFrom.Id +
                    "'})-[e: " + relation.RelationName + " ]->(y: " + relation.PointsTo.Type + " {ID: '" +
                    relation.PointsTo.Id + "'}) DELETE e";
        var resultSet = await RedisGraph.Query(GraphName, query);

        return resultSet != null && resultSet.Metrics.RelationshipsDeleted == 1;
    }

    public async Task UpdateAsync(TModel model)
    {
        var dto = ToTDto(model);
        await Collection.UpdateAsync(dto);
    }

    private async Task DeleteFromGraph(Guid id)
    {
        var model = new GraphEntityModel(id, _entityName);

        await DeleteGraphModelAsync(model);
    }

    /// <summary>
    ///     Sets the values for the <see cref="GraphEntityModel" /> from the specified node
    /// </summary>
    /// <param name="graphEntity"></param>
    /// <param name="nd"></param>
    protected static void SetGraphModelValues(GraphEntityModel graphEntity, Node nd)
    {
        var props = nd.Properties;
        var id = props["ID"];
        var type = props["Type"];
        var name = props["Name"];
        //graphEntity.Name = id.ToString();
        graphEntity.Id = Guid.Parse(id.ToString());
        graphEntity.Type = type.ToString();
        graphEntity.Name = name.ToString();
    }

    protected TDto ToTDto(TModel model)
    {
        return model.ToDto() as TDto ?? throw new MappingException(typeof(TModel), typeof(TDto));
    }

    protected TModel ToTModel(TDto dto)
    {
        return dto.ToModel() as TModel ?? throw new MappingException(typeof(TDto), typeof(TModel));
    }

    private async Task<bool> ObjectExistsOnGraph(GraphEntityModel graphModel)
    {
        var query = "match (x:" + _entityName + " {ID: '" + graphModel.Id + "'}) return x";
        var resultSet = await RedisGraph.Query(GraphName, query);
        return resultSet != null && resultSet.Results.Count > 0;
    }
}