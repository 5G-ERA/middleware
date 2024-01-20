using System;
using InfluxDB.Client.Api.Domain;
using InfluxDB.Client.Writes;

namespace Middleware.DataAccess.Repositories.Abstract;
public interface IInfluxRepository<TModel, TDto> where TModel : class
{
    public Task AddOrgAsync(string organisationName);
    public Task<Bucket> AddBucketAsync(string bucketName, int retentionTime);
    public Task AddPointAsync(PointData point, string bucketName, string org);
    /// <summary>
    /// Adds a new object to the data store
    /// </summary>
    /// <param name="model">Model of an object to be added</param>
    /// <returns></returns>
    Task<TModel?> AddAsync(TModel model);
    Task<TModel?> GetStatusByIdAsync(Guid id);
    Task<List<TModel>> GetAllAsync();
    Task GetAllAsyncTest();
}