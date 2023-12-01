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
using Neo4j.Driver;
using System.Reflection.Emit;
using Microsoft.Extensions.Logging;
using Elasticsearch.Net;
using System.Reflection;
using Neo4j.Driver.Preview.Mapping;
using System.Data;
using k8s;
using k8s.KubeConfigModels;
using StackExchange.Redis;
using System.Collections.Generic;
using System.Xml.Linq;

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

    protected readonly Microsoft.Extensions.Logging.ILogger Logger;

    protected IRedisGraphClient RedisGraph { get; }

    protected readonly IDriver Driver;

    public RedisRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, bool isWritableToGraph,
        Microsoft.Extensions.Logging.ILogger logger, IDriver driver)
    {
        RedisGraph = redisGraph;
        _entityName = typeof(TModel).GetModelName().ToUpper();
        Collection = provider.RedisCollection<TDto>();
        IsWritableToGraph = isWritableToGraph;
        Logger = logger;
        Driver = driver;
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
            await AddGraphAsyncTest(newGraphModel);
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
            await AddGraphAsyncTest(graphModel);
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

        using (var session = Driver.AsyncSession())
        {
            try
            {
                // Create a transaction to execute the Cypher query
                using (var transaction = await session.BeginTransactionAsync())
                {
                    var query = RelationDirection.Outgoing == direction
                        ? $"MATCH (n1:" + entityDef + ")-[r:" + relationName + "]->(n2) RETURN *"
                        : $"MATCH (n1:" + entityDef + ")<-[r:" + relationName + "]-(n2) RETURN *";

                    // Execute the Cypher query asynchronously
                    var result = await transaction.RunAsync(query);

                    while (await result.FetchAsync()) 
                    {
                        var record = result.Current;

                        var n1 = record.Values["n1"];
                        GraphEntityModel n1GraphModel = null;
                        if (n1 is Neo4j.Driver.INode node1)
                        {
                            var n1Id = node1.Properties["ID"].ToString();
                            var n1Name = node1.Properties["Name"].ToString();
                            var n1Type = node1.Properties["Type"].ToString();
                            n1GraphModel = new GraphEntityModel
                            {
                                Id = Guid.Parse(n1Id!),
                                Type = n1Type!,
                                Name = n1Name!

                            };
                        }
                        var n2 = record.Values["n2"];
                        GraphEntityModel n2GraphModel = null;
                        if (n2 is Neo4j.Driver.INode node2)
                        {
                            var n2Id = node2.Properties["ID"].ToString();
                            var n2Name = node2.Properties["Name"].ToString();
                            var n2Type = node2.Properties["Type"].ToString();
                            n2GraphModel = new GraphEntityModel
                            {
                                Id = Guid.Parse(n2Id!),
                                Type = n2Type!,
                                Name = n2Name!

                            };
                        }      
                        RelationModel relationModel = null;
                        var r = record.Values["r"];
                        if (r is IRelationship relationship)
                        {
                            var relType = relationship.Type;
                            if (n1GraphModel is null || n2GraphModel is null)
                            {
                                continue;
                            }
                            relationModel = new RelationModel
                            {
                                InitiatesFrom = n1GraphModel,
                                PointsTo = n2GraphModel,
                                RelationName = relType,
                            };
                        }                        
                        relationModels.Add(relationModel!);
                    }
                    if (relationModels.Count == 0) 
                    {
                        Logger.LogError("No relationship found.");
                        return null;
                    }
                }
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "An error occurred:");
                return null;
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
    public virtual async Task<Tuple<List<GraphEntityModel>, List<SimpleRelation>>> GetGraph()
    {
        var entities = new List<GraphEntityModel>();
        var relations = new List<SimpleRelation>();
        var entityIds = new List<Guid>();
        
        using (var session = Driver.AsyncSession())
        {
            try
            {
                // Create a transaction to execute the Cypher query
                using (var transaction = await session.BeginTransactionAsync())
                {
                    var query = "MATCH (n1) OPTIONAL MATCH (n1)-[r]->(n2) RETURN *";

                    // Execute the Cypher query asynchronously
                    var result = await transaction.RunAsync(query);

                    while (await result.FetchAsync())
                    {
                        var record = result.Current;

                        var n1 = record.Values["n1"];
                        GraphEntityModel n1GraphModel = null;
                        if (n1 is Neo4j.Driver.INode node1)
                        {
                            var n1Id = node1.Properties["ID"].ToString();
                            var n1Name = node1.Properties["Name"].ToString();
                            var n1Type = node1.Properties["Type"].ToString();
                            n1GraphModel = new GraphEntityModel
                            {
                                Id = Guid.Parse(n1Id!),
                                Type = n1Type!,
                                Name = n1Name!

                            };
                        }
                        if (entityIds.Contains(n1GraphModel!.Id) == false)
                        {
                            entities.Add(n1GraphModel);
                            entityIds.Add(n1GraphModel.Id);
                        }
                            
                        var n2 = record.Values["n2"];
                        GraphEntityModel n2GraphModel = null;
                        if (n2 is Neo4j.Driver.INode node2)
                        {
                            var n2Id = node2.Properties["ID"].ToString();
                            var n2Name = node2.Properties["Name"].ToString();
                            var n2Type = node2.Properties["Type"].ToString();
                            n2GraphModel = new GraphEntityModel
                            {
                                Id = Guid.Parse(n2Id!),
                                Type = n2Type!,
                                Name = n2Name!

                            };
                        }
                        if (n2GraphModel is not null && entityIds.Contains(n2GraphModel!.Id) == false)
                        {
                            entities.Add(n2GraphModel);
                            entityIds.Add(n2GraphModel.Id);
                        }

                        SimpleRelation relationModel = null;
                        var r = record.Values["r"];
                        if (r is IRelationship relationship)
                        {
                            var relType = relationship.Type;
                            if (n1GraphModel is null)
                            {
                                continue;
                            }
                            relationModel = new SimpleRelation
                            {
                                OriginatingId = n1GraphModel.Id,
                                RelationName = relType,
                                PointsToId = n2GraphModel.Id
                            };
                        }
                        if (n1GraphModel is not null && n2GraphModel is not null && relations.Contains(relationModel) == false)
                        {
                            relations.Add(relationModel);
                        }                      
                    }
                }
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "An error occurred:");
            }
        }
        return new Tuple<List<GraphEntityModel>, List<SimpleRelation>>(entities, relations);
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

        using (var session = Driver.AsyncSession())
        {
            try
            {
                // Create a transaction to execute the Cypher query
                using (var transaction = await session.BeginTransactionAsync())
                {
                    var query = $"CREATE (n: " + model.Type + " {ID: '" + model.Id + "', Type: '" + model.Type +
                    "', Name: '" + model.Name + "'})";

                    // Execute the Cypher query asynchronously
                    var result = await transaction.RunAsync(query);

                    // Commit the transaction
                    await transaction.CommitAsync();
                }
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "An error occurred:");
                return false;
            }
        }
        return true;
    }

    public virtual async Task<bool> AddGraphAsyncTest(GraphEntityModel model) 
    {
        /*IResultCursor result;

        await using var tx = Driver.AsyncSession();

        var cypher = $"CREATE (n: " + model.Type + " {ID: '" + model.Id + "', Type: '" + model.Type +
                    "', Name: '" + model.Name + "'})";

        if (model != null)
        {
            result = await tx.RunAsync(cypher);
        }
        else
        {
            result = await tx.RunAsync(cypher);
        }

        return await result?.ToListAsync();*/

        /*await using var session = Driver.AsyncSession();
        return await session.ExecuteWriteAsync(
            async tx =>
            {
                var query = $"CREATE (n: " + model.Type + " {ID: '" + model.Id + "', Type: '" + model.Type +
                    "', Name: '" + model.Name + "'})";
                var result = await tx.RunAsync(query);
                return await result.ToListAsync(r => r[0].As<string>());
            });*/

        await using var session = Driver.AsyncSession();

        var query = $"CREATE (n: " + model.Type + " {ID: '" + model.Id + "', Type: '" + model.Type +
                    "', Name: '" + model.Name + "'})";

        await session.ExecuteWriteAsync(async tx =>
        {
            var result = await tx.RunAsync(query);
            return result;
        });

        return true;


        /*var query = $"CREATE (n: " + model.Type + " {ID: '" + model.Id + "', Type: '" + model.Type +
                    "', Name: '" + model.Name + "'})";*/

        //var reader = await session.RunAsync(query);

        //await reader.ConsumeAsync();

        //return true;
    }

    /// <summary>
    ///     Creating a new relation between two models
    /// </summary>
    /// <param name="relation"></param>
    /// <returns></returns>
    public virtual async Task<bool> AddRelationAsync(RelationModel relation)
    {
        using (var session = Driver.AsyncSession())
        {
            try
            {
                // Create a transaction to execute the Cypher query
                using (var transaction = await session.BeginTransactionAsync())
                {
                    var query = "MATCH (n1: " + relation.InitiatesFrom.Type + " {ID: '" + relation.InitiatesFrom.Id +
                    "'}), (n2: " + relation.PointsTo.Type + " {ID: '" + relation.PointsTo.Id +
                    "'}) CREATE (n1)-[r:" + relation.RelationName + "]->(n2) ";

                    // Execute the Cypher query asynchronously
                    var result = await transaction.RunAsync(query);

                    // Commit the transaction
                    await transaction.CommitAsync();
                }
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "An error occurred:");
                return false;
            }
        }
        return true;
    }

    /// <summary>
    ///     Removing a model from RedisGraph db
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    public virtual async Task<bool> DeleteGraphModelAsync(GraphEntityModel model)
    {
        model.Type = _entityName;

        using (var session = Driver.AsyncSession())
        {
            try
            {
                // Create a transaction to execute the Cypher query
                using (var transaction = await session.BeginTransactionAsync())
                {
                    var query = "MATCH (n: " + model.Type + " {ID: '" + model.Id + "'}) DELETE n";

                    // Execute the Cypher query asynchronously
                    var result = await transaction.RunAsync(query);

                    // Commit the transaction
                    await transaction.CommitAsync();
                }
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "An error occurred:");
                return false;
            }
        }
        return true;
    }

    /// <summary>
    ///     Removing a relation from the RedisGraph db
    /// </summary>
    /// <param name="relation"></param>
    /// <returns></returns>
    public virtual async Task<bool> DeleteRelationAsync(RelationModel relation)
    {
        using (var session = Driver.AsyncSession())
        {
            try
            {
                // Create a transaction to execute the Cypher query
                using (var transaction = await session.BeginTransactionAsync())
                {
                    var query = "MATCH (n1: " + relation.InitiatesFrom.Type + " {ID: '" + relation.InitiatesFrom.Id +
                    "'})-[r: " + relation.RelationName + " ]->(n2: " + relation.PointsTo.Type + " {ID: '" +
                    relation.PointsTo.Id + "'}) DELETE r";

                    // Execute the Cypher query asynchronously
                    var result = await transaction.RunAsync(query);

                    // Commit the transaction
                    await transaction.CommitAsync();
                }
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "An error occurred:");
                return false;
            }
        }
        return true;
    }

    public async Task UpdateAsync(TModel model)
    {
        var dto = ToTDto(model);
        await Collection.InsertAsync(dto);
        await UpdateGraphAsync(model);
    }

    private async Task UpdateGraphAsync(TModel model) 
    {
        using (var session =    Driver.AsyncSession())
        {
            try
            {
                // Create a transaction to execute the Cypher query
                using (var transaction = await session.BeginTransactionAsync())
                {
                    var query = "MATCH(n1: " + _entityName + " ) WHERE n1.ID = '" + model.Id + "' SET n1.Name = '" + model.Name + "' RETURN n1";

                    // Execute the Cypher query asynchronously
                    var result = await transaction.RunAsync(query);

                    // Commit the transaction
                    await transaction.CommitAsync();
                }
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "An error occurred:");
            }
        }     
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
        bool exists = false;

        using (var session = Driver.AsyncSession())
        {
            try
            {
                // Create a transaction to execute the Cypher query
                using (var transaction = await session.BeginTransactionAsync())
                {
                    var query = "MATCH (n1:" + _entityName + " {ID: '" + graphModel.Id + "'}) RETURN n1";

                    // Execute the Cypher query asynchronously
                    var result = await transaction.RunAsync(query);

                    while (await result.FetchAsync())
                    {
                        var record = result.Current;

                        if (record.Keys.Count() > 0)
                        {
                            exists = true;
                        }
                        else { exists = false; }                                  
                    }
                }
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "An error occurred:");
            }
        }
        return exists;
    }
}