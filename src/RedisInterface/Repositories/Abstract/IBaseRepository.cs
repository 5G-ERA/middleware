namespace Middleware.RedisInterface.Repositories;

public interface IBaseRepository<T> where T : class
{
    List<T> ExecuteLuaQuery(string queryName);
}