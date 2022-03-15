namespace Middleware.RedisInterface.Repositories;

public interface IBaseRepository<T> where T : class
{
    Task<T> AddAsync(T model);
    Task<T> GetByIdAsync(Guid id);
    Task<List<T>> GetAllAsync(Guid id);
    Task<bool> DeleteByIdAsync(Guid id);
}