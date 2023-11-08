using System.Collections.Immutable;
using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface ILocationRepository : IRedisRepository<LocationModel, LocationDto>
{
    Task<LocationModel?> PatchAsync(Guid id, LocationModel patch);

    /// <summary>
    ///     Get Location by name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<LocationModel?> GetByNameAsync(string name);

    /// <summary>
    ///     Filter out locations that are not busy (have no instances deployed)
    /// </summary>
    /// <param name="locationsToCheck">List of Locations to be checked</param>
    /// <returns></returns>
    Task<List<LocationModel>> FilterFreeLocationsAsync(IReadOnlyList<LocationModel> locationsToCheck);

    /// <summary>
    ///     Order the given list of locations by the number of deployed instances in ascending order.
    /// </summary>
    /// <param name="locationsToCheck"></param>
    /// <param name="descending">Should the order be descending</param>
    /// <returns></returns>
    Task<List<LocationModel>> OrderLocationsByUtilizationAsync(List<LocationModel> locationsToCheck,
        bool descending = false);

    /// <summary>
    ///     Get the number of deployed instances
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<int> GetDeployedInstancesCountAsync(string name);

    Task<int> GetDeployedInstancesCountAsync(Guid id);

    Task<bool> IsBusyAsync(string name);

    Task<bool> IsBusyAsync(Guid id);
    Task<ImmutableList<LocationModel>> GetLocationsByOrganizationAsync(string organization);

    Task<(bool, LocationModel?)> ExistsAsync(string name);

    Task<(bool, LocationModel?)> ExistsAsync(Guid id);
}