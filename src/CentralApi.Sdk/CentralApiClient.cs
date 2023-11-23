using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.CentralApi.Sdk.Client;
using Middleware.CentralApi.Sdk.Options;
using Middleware.Models.Domain;

namespace Middleware.CentralApi.Sdk;

/// <summary>
///     Central API Middleware client
/// </summary>
public class CentralApiClient : ICentralApiClient
{
    private readonly LocationApiAccessOptions _accessOptions;
    private readonly ICentralApi _api;
    private readonly ILogger<CentralApiClient> _logger;

    public CentralApiClient(ICentralApi api, IOptions<LocationApiAccessOptions> accessOptions,
        ILogger<CentralApiClient> logger)
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

    /// <inheritdoc />
    public async Task<LocationsResponse?> GetAvailableLocations(string orgName)
    {
        _logger.LogDebug("Entered GetAvailableLocations(string orgName)");

        var response = await _api.GetAvailableLocations(orgName);

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

    public async Task SetStatus(CloudEdgeStatusRequest request, Guid id)
    {
        if (request == null) throw new ArgumentNullException(nameof(request));
        if (id == Guid.Empty) throw new ArgumentNullException(nameof(id));
        if (request.Type == string.Empty) throw new ArgumentNullException(nameof(request.Type));
        request.IsOnline = true;

        await _api.SetStatus(request, id);
    }
}
