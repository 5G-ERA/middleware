namespace Middleware.RedisInterface.Repositories;

public interface IBaseRepository<T> where T : class
{
    Task<List<T>> ExecuteLuaQueryAsync(string queryName);
}