using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Contracts.Responses;

namespace Middleware.CentralApi.Sdk;

public interface ICentralApiClient
{
    /// <summary>
    /// Gets all available (online) Middleware locations  
    /// </summary>
    /// <returns></returns>
    Task<LocationsResponse?> GetAvailableLocations();

    /// <summary>
    /// Sets specified location available (online) to be communicated to 
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    Task<LocationResponse?> RegisterLocation(RegisterRequest request);

    /// <summary>
    /// Sets specified location unavailable (offline)
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    Task DeRegisterLocation(RegisterRequest request);
}