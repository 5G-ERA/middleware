using Middleware.Common.Enums;

namespace Middleware.CentralApi.Auth;

public interface IApiKeyService
{
    string GenerateApiKey(ApiKeyUserType userType);
    bool ValidateApiKeyFormat(string apiKey);
    Task<Dictionary<string, Guid>> GetClientKeys();
}