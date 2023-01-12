using Middleware.Common.Enums;
using Middleware.Common.Models;

namespace Middleware.Common.Repositories;

public interface IBaseRepository<T> where T : class
{
    /// <summary>
    /// Adds a new object to the data store
    /// </summary>
    /// <param name="model">Model of an object to be added</param>
    /// <returns></returns>
    Task<T> AddAsync(T model);
    /// <summary>
    /// Adds a new object to the data store and specify the <see cref="Guid"/> generation method
    /// </summary>
    /// <param name="model">Model of an object to be added</param>
    /// <param name="guidProvider">Function providing guid to be used to save data, when not overloaded the <see cref="Guid.NewGuid()"/> method is used</param>
    /// <returns></returns>
    Task<T> AddAsync(T model, Func<Guid> guidProvider);
    /// <summary>
    /// Get the object by its id
    /// </summary>
    /// <param name="id">Identifier of the object</param>
    /// <returns></returns>
    Task<T> GetByIdAsync(Guid id);
    /// <summary>
    /// Get all objects of the specified type from the data store
    /// </summary>
    /// <returns></returns>
    Task<List<T>> GetAllAsync();
    /// <summary>
    /// Delete object from the data store by its id
    /// </summary>
    /// <param name="id">Identifier of the object</param>
    /// <returns></returns>
    Task<bool> DeleteByIdAsync(Guid id);
    /// <summary>
    /// Get relation for the specified object by the name of the relation
    /// </summary>
    /// <param name="id">Identifier of the object</param>
    /// <param name="relationName">Name of the relation</param>
    /// <param name="direction">Direction of the relation, outgoing from the object or incoming to the object</param>
    /// <returns></returns>
    Task<List<RelationModel>> GetRelation(Guid id, string relationName, RelationDirection direction = RelationDirection.Outgoing);
    /// <summary>
    /// Get relations for the specified object by the names of the relations
    /// </summary>
    /// <param name="id">Identifier of the object</param>
    /// <param name="relationNames">Names of the relations</param>
    /// <returns></returns>
    Task<List<RelationModel>> GetRelations(Guid id, List<string> relationNames);

    Task<bool> AddGraphAsync(GraphEntityModel model);
    Task<bool> AddRelationAsync(RelationModel relation);

    Task<bool> DeleteGraphModelAsync(GraphEntityModel model);

    Task<bool> DeleteRelationAsync(RelationModel relation);

    Task<Dictionary<string, List<RedisGraphResult>>> GetAllRelations();
}