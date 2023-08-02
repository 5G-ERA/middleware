using Microsoft.Extensions.Caching.Memory;

namespace Middleware.CentralApi.Auth;

internal class CacheService : ICacheService
{
    private readonly IApiKeyService _apiKeyService;
    private readonly IMemoryCache _memoryCache;


    public CacheService(IMemoryCache memoryCache, IApiKeyService apiKeyService)
    {
        _memoryCache = memoryCache;
        _apiKeyService = apiKeyService;
    }

    public async ValueTask<Guid> GetClientIdFromApiKey(string apiKey)
    {
        if (!_memoryCache.TryGetValue<Dictionary<string, Guid>>("Authentication_ApiKeys", out var internalKeys))
        {
            internalKeys = await _apiKeyService.GetClientKeys();

            _memoryCache.Set("Authentication_ApiKeys", internalKeys);
        }

        if (!internalKeys.TryGetValue(apiKey, out var clientId)) return Guid.Empty;

        return clientId;
    }
}