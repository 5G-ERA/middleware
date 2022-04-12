using Middleware.Common.Models;

namespace Middleware.Common.Repositories;

public interface IBaseRepository<T> where T : class
{
    Task<T> AddAsync(T model);
    Task<T> GetByIdAsync(Guid id);
    Task<List<T>> GetAllAsync();
    Task<bool> DeleteByIdAsync(Guid id);

    Task<List<RelationModel>> GetRelation(Guid id, string relationName);

    Task<List<RelationModel>> GetRelations(Guid id, List<string> relationNames);
}