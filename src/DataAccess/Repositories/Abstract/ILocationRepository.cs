using System.Collections.Immutable;
using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface ILocationRepository : IRedisRepository<Location, LocationDto>
{
    Task<Location?> PatchAsync(Guid id, Location patch);

    /// <summary>
    ///     Get Location by name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<Location?> GetByNameAsync(string name);

    /// <summary>
    ///     Filter out locations that are not busy (have no instances deployed)
    /// </summary>
    /// <param name="locationsToCheck">List of Locations to be checked</param>
    /// <returns></returns>
    Task<List<Location>> FilterFreeLocationsAsync(IReadOnlyList<Location> locationsToCheck);

    /// <summary>
    ///     Order the given list of locations by the number of deployed instances in ascending order.
    /// </summary>
    /// <param name="locationsToCheck"></param>
    /// <param name="descending">Should the order be descending</param>
    /// <returns></returns>
    Task<List<Location>> OrderLocationsByUtilizationAsync(List<Location> locationsToCheck,
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
    Task<ImmutableList<Location>> GetLocationsByOrganizationAsync(string organization);

    Task<(bool, Location?)> ExistsAsync(string name);

    Task<(bool, Location?)> ExistsAsync(Guid id);
    Task<CloudEdgeStatusResponse> GetCloudOnlineStatusLastUpdatedTimeAsync(Guid locationId);

    /// <summary>
    ///     Change online status of cloud by cloud Id.
    /// </summary>
    /// <param name="cloudId"></param>
    /// <param name="isOnline"></param>
    /// <returns></returns>
    Task SetCloudOnlineStatusAsync(Guid cloudId, bool isOnline);
}