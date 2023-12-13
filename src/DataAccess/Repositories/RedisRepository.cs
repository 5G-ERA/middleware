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
using KeyValuePair = Middleware.Models.Domain.KeyValuePair;

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
            ? "MATCH (x) MATCH (y) WHERE (x)-[:" + relationName + "]->(y:" + entityDef + " ) RETURN x,y"
            : "MATCH (x:" + entityDef + ") MATCH(y) WHERE(x) -[:" + relationName + "]->(y) RETURN x, y";
        ResultSet? resultSet;
        try
        {
            resultSet = await RedisGraph.Query(GraphName, query);
        }
        catch (FormatException) //BB 2023.07.07 - when no results are found the library throws FormatException
        {
            return relationModels;
        }

        relationModels = ExtractFullRelation(resultSet, relationName);

        return relationModels;
    }

    // Designed to check if speific relation between two known entities exist
    public virtual async Task<RelationModel> GetOneRelation(RelationModel relation)
    {
        if (string.IsNullOrWhiteSpace(relation.RelationName))
        {
            throw new ArgumentException($"'{nameof(relation.RelationName)}' cannot be null or whitespace.",
                nameof(relation.RelationName));
        }

        var relationName = relation.RelationName;
        relationName = relationName.ToUpper();

        var relationModels = new List<RelationModel>();
        var finalRelationModel = new RelationModel();

        var query = "MATCH (x:ROBOT)-[r:" + relationName +"]->(y:LOCATION) where (x)-[r]->(y) and x.ID='" + relation.InitiatesFrom.Id +
            "' and y.ID='" + relation.PointsTo.Id + "' return x,y,r";
        //var query33 =  "MATCH (x:ROBOT)-[r:CAN_REACH]->(y:LOCATION) RETURN x,y,r";
        ResultSet? resultSet;

        try
        {
            resultSet = await RedisGraph.Query(GraphName, query);
        }
        catch (FormatException) //BB 2023.07.07 - when no results are found the library throws FormatException
        {
            return finalRelationModel;
        }

        relationModels = ExtractFullRelation(resultSet, relationName);
        finalRelationModel = relationModels.FirstOrDefault();
        return finalRelationModel;
    }

    // will return all relations which contain passed relation name
    public virtual async Task<List<RelationModel>> GetRelationsWithName(string relationName)
    {
        if (string.IsNullOrWhiteSpace(relationName))
        {
            throw new ArgumentException($"'{nameof(relationName)}' cannot be null or whitespace.",
                nameof(relationName));
        }

        relationName = relationName.ToUpper();
        var relationModels = new List<RelationModel>();
        var query33 = "MATCH (x)-[r:"+ relationName + "]->(y) RETURN x,y,r";

        ResultSet? resultSet;
        try
        {
            resultSet = await RedisGraph.Query(GraphName, query33);
        }
        catch (FormatException) //BB 2023.07.07 - when no results are found the library throws FormatException
        {
            return relationModels;
        }

        relationModels = ExtractFullRelation(resultSet, relationName);

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
        try
        {
            var query = "CREATE (x: " + model.Type + " {ID: '" + model.Id + "', Type: '" + model.Type +
                        "', Name: '" + model.Name + "'})";
            var resultSet = await RedisGraph.Query(GraphName, query);

            return resultSet != null && resultSet.Metrics.NodesCreated == 1;
        }
        catch (Exception)
        {
            // do nothing, it still ads it :/   
            return true;
        }
    }

    /// <summary>
    ///     Creating a new relation between two models
    /// </summary>
    /// <param name="relation"></param>
    /// <returns></returns>
    public virtual async Task<bool> AddRelationAsync(RelationModel relation)
    {
        // check if relation exist
        var relationExist = await GetOneRelation(relation);

        if (relationExist != null)
        {
            if (relationExist.InitiatesFrom.Id == relation.InitiatesFrom.Id)
            {
                //Relation exist
                // Update relation

                string querySetAttributes = GetQueryAtributesToUpdate(relation);

                var query = "MATCH (x: " + relation.InitiatesFrom.Type + " {ID: '" + relation.InitiatesFrom.Id +
                             "'})-[r: " + relation.RelationName + "]->(c: " + relation.PointsTo.Type +
                             " {ID: '" + relation.PointsTo.Id + "'}) SET " + querySetAttributes;

                var resultSet = await RedisGraph.Query(GraphName, query);

                return resultSet != null && resultSet.Metrics.RelationshipsCreated >= 1;
            }
            else
            {
                //Relation does not exist
                // Create relation
                string queryAttributes = GetQueryAtributesToCreate(relation);

                var query = "MATCH (x: " + relation.InitiatesFrom.Type + " {ID: '" + relation.InitiatesFrom.Id +
                            "'}), (c: " + relation.PointsTo.Type + " {ID: '" + relation.PointsTo.Id +
                            "'}) CREATE (x)-[:" + relation.RelationName + queryAttributes + "]->(c) ";

                var resultSet = await RedisGraph.Query(GraphName, query);

                return resultSet != null && resultSet.Metrics.RelationshipsCreated >= 1;
            }
        }
        else
        {
            //Relation does not exist
            // Create relation
            string queryAttributes = GetQueryAtributesToCreate(relation);

            var query = "MATCH (x: " + relation.InitiatesFrom.Type + " {ID: '" + relation.InitiatesFrom.Id +
                        "'}), (c: " + relation.PointsTo.Type + " {ID: '" + relation.PointsTo.Id +
                        "'}) CREATE (x)-[:" + relation.RelationName + queryAttributes + "]->(c) ";

            var resultSet = await RedisGraph.Query(GraphName, query);

            return resultSet != null && resultSet.Metrics.RelationshipsCreated >= 1;
        }
    }
    /// <summary>
    ///     Creating a new relation between two models
    /// </summary>
    /// <param name="relation"></param>
    /// <returns></returns>
    /// 

    public virtual async Task<bool> UpdateRelationAsync(RelationModel relation)
    {
        var relationExist = await GetOneRelation(relation);
        if (relationExist != null)
        {
            if(relationExist.InitiatesFrom.Id == relation.InitiatesFrom.Id)
            {
                //Relation exist
                // Update relation

                string querySetAttributes = GetQueryAtributesToUpdate(relation);      

                var query = "MATCH (x: " + relation.InitiatesFrom.Type + " {ID: '" + relation.InitiatesFrom.Id +
                             "'})-[r: " + relation.RelationName + "]->(c: " + relation.PointsTo.Type +
                             " {ID: '" + relation.PointsTo.Id + "'}) SET " + querySetAttributes;

                var resultSet = await RedisGraph.Query(GraphName, query);

                return resultSet != null && resultSet.Metrics.RelationshipsCreated >= 1;
            } else { return false; }
        }
        else
        {
            return false;
        }
    }

    private string GetQueryAtributesToCreate(RelationModel relation)
    {
        var allAttributes = relation.RelationAttributes;
        string queryAttributes = "";

        // create following pattern as string
        // {key1:'val1',key2: 'val2'}
        if (allAttributes != null)
        {
            bool atLeastOneValidAttribute = false;
            queryAttributes = "{";
            foreach (KeyValuePair item in allAttributes)
            {
                if (item.Key != null && item.Value != null && item.Key != string.Empty && item.Value != string.Empty)
                {
                    queryAttributes += item.Key + ":'" + item.Value + "',";
                    atLeastOneValidAttribute = true;
                }
            }
            queryAttributes = queryAttributes.Remove(queryAttributes.Length - 1);
            queryAttributes += "}";
            if (atLeastOneValidAttribute == false)
            {
                // if did not find any valid key value pair, do not add any attributes;
                queryAttributes = "";
            }
        }
        return queryAttributes;
    }
    private string GetQueryAtributesToUpdate(RelationModel relation)
    {
        var allAttributes = relation.RelationAttributes;
        string queryAttributes = "";

        // create following pattern as string
        // r.time_udated1 = 'new_Value24', r.time_udated2 = 'new_Value234', r.time_udated3 = 'new_Value2345'"
        if (allAttributes != null)
        {
            bool atLeastOneValidAttribute = false;
            
            foreach (KeyValuePair item in allAttributes)
            {
                if (item.Key != null && item.Value != null && item.Key != string.Empty && item.Value != string.Empty)
                {
                    queryAttributes += "r." + item.Key + "= '" + item.Value + "',";
                    atLeastOneValidAttribute = true;
                }
            }
            queryAttributes = queryAttributes.Remove(queryAttributes.Length - 1);

            if (atLeastOneValidAttribute == false)
            {
                // if did not find any valid key value pair, do not add any attributes;
                queryAttributes = "";
            }
        }
        return queryAttributes;
    }



    //GRAPH.QUERY Cars 'MERGE (honda:Cars {id: "c12345"}) MERGE (driver1:Person {id: "d12345"}) MERGE (honda)-[r1:rider {id: "r12345", via: "uber"}]->(driver1)'

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
        await Collection.InsertAsync(dto);
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
        try
        {
            var query = "match (x:" + _entityName + " {ID: '" + graphModel.Id + "'}) return x";
            var resultSet = await RedisGraph.Query(GraphName, query);
            return resultSet != null && resultSet.Results.Count > 0;
        }
        catch (Exception)
        {
            return false;
        }
    }

    private List<RelationModel> ExtractRelations(ResultSet resultSet, string relationName)
    {
        var relationModels = new List<RelationModel>();
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

    private List<RelationModel> ExtractFullRelation(ResultSet resultSet, string relationName)
    {
        var relationModels = new List<RelationModel>();
        // BB: 12.12.2023
        // example of query in order to work correctly
        // var query33 = "MATCH (x:ROBOT)-[r:CAN_REACH]->(y:LOCATION) RETURN x,y,r";
        // We are using the loop with 3 nested loops to retrieve the values from the graph
        // The values are structured in the following way:
        // First result contains the information about the objects that the relation initiates from
        // Second result contains the information about the objects that the relation is pointing to
        // Third result contains information about propertis of the relation atributes
        // This structure will be universal for the explanation of all the queries on the redis graph

        for (var i = 0; i < resultSet.Results.Count; i++)
        {
            var res = resultSet.Results.ElementAt(i);
            if (i % 3 == 0)
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
            else if (i % 3 == 1)
            {
                foreach (var node in res.Value)
                {
                    var idxTmp = res.Value.IndexOf(node);
                    var relationModel = relationModels[idxTmp];
                    if (node is Node nd) SetGraphModelValues(relationModel.PointsTo, nd);
                }
            }
            else
            {
                foreach (var relation in res.Value)
                {
                    var idxTmp = res.Value.IndexOf(relation);
                    var relationModel = relationModels[idxTmp];
                    if (relation is Relation nd)
                        relationModel.RelationAttributes = GetRelationAttributes(nd);
                }
            }
        }
        return relationModels;
    }

    private List<KeyValuePair> GetRelationAttributes(Relation nd)
    {
        var retVal = new List<KeyValuePair>();
        foreach (var item in nd.Properties)
        {
            var key = item.Key;
            var value = item.Value;
            var kv = new KeyValuePair
            {
                Key = key,
                Value = value.ToString()
            };
            retVal.Add(kv);
        }
        return retVal;
    }
}