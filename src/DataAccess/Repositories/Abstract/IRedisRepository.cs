using System.Linq.Expressions;
using Redis.OM.Searching;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface IRedisRepository<T>
{
    /// <summary>
    /// Adds a new object to the data store
    /// </summary>
    /// <param name="model">Model of an object to be added</param>
    /// <returns></returns>
    Task<T> AddAsync(T model);
    /// <summary>
    /// Get the object by its id
    /// </summary>
    /// <param name="id">Identifier of the object</param>
    /// <returns></returns>
    Task<T?> GetByIdAsync(Guid id);

    /// <summary>
    /// Get all objects of the specified type from the data store
    /// </summary>
    /// <returns></returns>
    Task<List<T>> GetAllAsync();
    /// <summary>
    /// Delete object from the data store
    /// </summary>
    /// <param name="model">object to delete</param>
    /// <returns></returns>
    Task DeleteAsync(T model);
    /// <summary>
    /// Find elements matching predicate
    /// </summary>
    /// <param name="predicate"></param>
    /// <returns></returns>
    Task<List<T>> FindAsync(Expression<Func<T, bool>> predicate);
    /// <summary>
    /// Find single element matching a predicate
    /// </summary>
    /// <param name="predicate"></param>
    /// <returns></returns>
    Task<T?> FindSingleAsync(Expression<Func<T, bool>> predicate);
    /// <summary>
    /// Filter the elements matching the predicate, do not execute the query
    /// </summary>
    /// <param name="predicate"></param>
    /// <returns></returns>
    IRedisCollection<T> FindQuery(Expression<Func<T, bool>> predicate);
}
