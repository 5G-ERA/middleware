using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.Models.Domain;
using Refit;

namespace Middleware.CentralApi.Sdk.Client;

public interface ICentralApi
{
    [Get("/api/v1/locations")]
    Task<ApiResponse<LocationsResponse>> GetAvailableLocations(string organization);
    [Post("/api/v1/register")]
    Task<ApiResponse<LocationResponse>> RegisterLocation([Body]RegisterRequest request);
    [Post("")]
    Task DeRegisterLocation([Body]RegisterRequest request);

    [Post("/api/v1/locations/status/{id}")]
    Task SetStatus([Body] CloudEdgeStatusRequest request, Guid id);
}