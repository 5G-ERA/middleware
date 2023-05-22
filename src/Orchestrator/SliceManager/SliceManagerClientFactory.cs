using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Orchestrator.SliceManager.Contracts;
using Refit;

namespace Middleware.Orchestrator.SliceManager;

internal class SliceManagerClientFactory
{
    private readonly IHttpClientFactory _httpClientFactory;
    private readonly IOptions<SliceConfig> _sliceConfig;

    public SliceManagerClientFactory(IHttpClientFactory httpClientFactory, IOptions<SliceConfig> sliceConfig)
    {
        _httpClientFactory = httpClientFactory;
        _sliceConfig = sliceConfig;
    }

    public bool IsSlicingAvailable()
    {
        return _sliceConfig.Value is not null
               && string.IsNullOrWhiteSpace(_sliceConfig.Value.Hostname)
               && Uri.IsWellFormedUriString(_sliceConfig.Value.Hostname, UriKind.RelativeOrAbsolute);
    }

    public ISliceManager CreateSliceManagerClient()
    {
        if (IsSlicingAvailable())
            return null;

        var client = RestService.For<ISliceManagerApi>(_sliceConfig.Value.Hostname);

        return new SliceManager(client, IsSlicingAvailable());
    }
}