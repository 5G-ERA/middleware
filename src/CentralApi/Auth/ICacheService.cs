namespace Middleware.CentralApi.Auth;

internal interface ICacheService
{
    ValueTask<Guid> GetClientIdFromApiKey(string apiKey);
}