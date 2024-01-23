using InfluxDB.Client.Api.Domain;
using InfluxDB.Client.Writes;

namespace Middleware.DataAccess.Repositories.Abstract;
public interface IInfluxRepository<TModel> where TModel : class
{
    /// <summary>
    /// Adds a new object to the data store
    /// </summary>
    /// <param name="model">Model of an object to be added</param>
    /// <returns></returns>
    Task<TModel?> AddAsync(TModel model);
    Task<TModel?> GetStatusByIdAsync(Guid id);
    Task<List<TModel>> GetAllAsync();
}