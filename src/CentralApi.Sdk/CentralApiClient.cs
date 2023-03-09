using Microsoft.Extensions.Logging;
using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.CentralApi.Sdk.Client;
using Middleware.CentralApi.Sdk.Options;

namespace Middleware.CentralApi.Sdk;

/// <summary>
/// Central API Middleware client
/// </summary>
public class CentralApiClient : ICentralApiClient
{
    private readonly ICentralApi _api;
    private readonly LocationApiAccessOptions _accessOptions;
    private readonly ILogger<CentralApiClient> _logger;

    internal CentralApiClient(ICentralApi api, LocationApiAccessOptions accessOptions, ILogger<CentralApiClient> logger)
    {
        _api = api;
        _accessOptions = accessOptions;
        _logger = logger;
    }
    public async Task<LocationsResponse> GetAvailableLocations()
    {
        _logger.LogDebug("Entered GetAvailableLocations");
        
        var org = _accessOptions.Organization;
        
        return await _api.GetAvailableLocations(org);
    }

    public async Task<LocationResponse> RegisterLocation(RegisterRequest request)
    {
        if (request == null) throw new ArgumentNullException(nameof(request));
        _logger.LogDebug("Entered RegisterLocation");

        return await _api.RegisterLocation(request);
    }

    public async Task DeRegisterLocation(RegisterRequest request)
    {
        _logger.LogDebug("Entered DeRegisterLocation");
        throw new NotImplementedException();
    }
}