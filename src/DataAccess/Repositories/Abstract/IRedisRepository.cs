using System.Linq.Expressions;
using Redis.OM.Searching;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface IRedisRepository<T> : IRelationRepository, IBaseRepository<T> where T : class
{
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
    /// <summary>
    /// Updates the existing entity
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    Task UpdateAsync(T model);
}
