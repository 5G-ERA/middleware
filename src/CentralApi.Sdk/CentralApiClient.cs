using Microsoft.Extensions.Logging;
using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.CentralApi.Sdk.Options;

namespace Middleware.CentralApi.Sdk;

/// <summary>
/// Central API Middleware client
/// </summary>
public class CentralApiClient: ICentralApiClient
{
    private readonly LocationApiAccessOptions _accessOptions;
    private readonly ILogger<CentralApiClient> _logger;

    public CentralApiClient(LocationApiAccessOptions accessOptions, ILogger<CentralApiClient> logger)
    {
        _accessOptions = accessOptions;
        _logger = logger;
    }
    public async Task<LocationsResponse> GetAvailableLocations()
    {
        _logger.LogDebug("Entered GetAvailableLocations");
        throw new NotImplementedException();
    }

    public async Task<LocationResponse> RegisterLocation(RegisterRequest request)
    {
        _logger.LogDebug("Entered RegisterLocation");
        throw new NotImplementedException();
    }

    public async Task DeRegisterLocation(RegisterRequest request)
    {
        _logger.LogDebug("Entered DeRegisterLocation");
        throw new NotImplementedException();
    }
}