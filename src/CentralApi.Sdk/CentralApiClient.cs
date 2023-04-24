using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
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

    public CentralApiClient(ICentralApi api, IOptions<LocationApiAccessOptions> accessOptions, ILogger<CentralApiClient> logger)
    {
        _api = api;
        _accessOptions = accessOptions.Value;
        _logger = logger;
    }
    public async Task<LocationsResponse?> GetAvailableLocations()
    {
        _logger.LogDebug("Entered GetAvailableLocations");
        
        var org = _accessOptions.Organization;
        var response = await _api.GetAvailableLocations(org);

        return response.IsSuccessStatusCode ? response.Content : null;
    }

    public async Task<LocationResponse?> RegisterLocation(RegisterRequest request)
    {
        if (request == null) throw new ArgumentNullException(nameof(request));
        _logger.LogDebug("Entered RegisterLocation");
        var response = await _api.RegisterLocation(request);

        return response.IsSuccessStatusCode ? response.Content : null;
    }

    public async Task DeRegisterLocation(RegisterRequest request)
    {
        _logger.LogDebug("Entered DeRegisterLocation");
        throw new NotImplementedException();
    }
}