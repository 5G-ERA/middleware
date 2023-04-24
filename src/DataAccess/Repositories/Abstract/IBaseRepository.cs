namespace Middleware.DataAccess.Repositories.Abstract;

public interface IBaseRepository<T> where T : class
{
    /// <summary>
    /// Adds a new object to the data store
    /// </summary>
    /// <param name="model">Model of an object to be added</param>
    /// <returns></returns>
    Task<T?> AddAsync(T model);

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
    Task<T?> GetByIdAsync(Guid id);

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
    /// Updates the value of the specified entity
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    Task UpdateAsync(T model);
}