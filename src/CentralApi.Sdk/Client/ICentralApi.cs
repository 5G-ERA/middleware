using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Contracts.Responses;
using Refit;

namespace Middleware.CentralApi.Sdk.Client;

internal interface ICentralApi
{
    [Get("/api/v1/locations")]
    Task<LocationsResponse> GetAvailableLocations(string organization);
    [Post("/api/v1/register")]
    Task<LocationResponse> RegisterLocation([Body]RegisterRequest request);
    [Post("")]
    Task DeRegisterLocation([Body]RegisterRequest request);
}