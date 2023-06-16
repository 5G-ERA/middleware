using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Orchestrator.SliceManager.Contracts;
using Middleware.RedisInterface.Sdk;
using Refit;

namespace Middleware.Orchestrator.SliceManager;

internal class SliceManagerClientFactory : ISliceManagerClientFactory
{
    private readonly IRedisInterfaceClient _redisInterfaceClient;
    private readonly IOptions<SliceConfig> _sliceConfig;

    public SliceManagerClientFactory(IOptions<SliceConfig> sliceConfig, IRedisInterfaceClient redisInterfaceClient)
    {
        _sliceConfig = sliceConfig;
        _redisInterfaceClient = redisInterfaceClient;
    }

    public bool IsSlicingAvailable()
    {
        return string.IsNullOrWhiteSpace(_sliceConfig.Value.Hostname) == false
               && Uri.IsWellFormedUriString(_sliceConfig.Value.Hostname, UriKind.RelativeOrAbsolute);
    }

    public ISliceManager CreateSliceManagerClient()
    {
        if (!IsSlicingAvailable())
            return null;

        var client = RestService.For<ISliceManagerApi>(_sliceConfig.Value.Hostname);

        return new SliceManager(client, IsSlicingAvailable(), _redisInterfaceClient);
    }
}