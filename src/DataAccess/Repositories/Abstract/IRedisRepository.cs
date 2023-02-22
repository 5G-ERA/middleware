using System.Linq.Expressions;
using Redis.OM.Searching;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface IRedisRepository<TModel, TDto> : IRelationRepository, IBaseRepository<TModel> where TModel : class
{
    /// <summary>
    /// Delete object from the data store
    /// </summary>
    /// <param name="model">object to delete</param>
    /// <returns></returns>
    Task DeleteAsync(TModel model);
    /// <summary>
    /// Find elements matching predicate
    /// </summary>
    /// <param name="predicate"></param>
    /// <returns></returns>
    Task<List<TModel>> FindAsync(Expression<Func<TDto, bool>> predicate);
    /// <summary>
    /// Find single element matching a predicate
    /// </summary>
    /// <param name="predicate"></param>
    /// <returns></returns>
    Task<TModel?> FindSingleAsync(Expression<Func<TDto, bool>> predicate);
    /// <summary>
    /// Filter the elements matching the predicate, do not execute the query
    /// </summary>
    /// <param name="predicate"></param>
    /// <returns></returns>
    IRedisCollection<TDto> FindQuery(Expression<Func<TDto, bool>> predicate);
    /// <summary>
    /// Updates the existing entity
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    Task UpdateAsync(TModel model);
}
