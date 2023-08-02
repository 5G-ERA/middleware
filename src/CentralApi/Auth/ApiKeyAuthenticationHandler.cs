using System.Security.Claims;
using System.Text.Encodings.Web;
using Microsoft.AspNetCore.Authentication;
using Microsoft.Extensions.Options;

namespace Middleware.CentralApi.Auth;

internal class ApiKeyAuthenticationHandler : AuthenticationHandler<ApiKeyAuthOptions>
{
    private readonly IApiKeyService _apiKeyService;
    private readonly ICacheService _cacheService;

    /// <inheritdoc />
    public ApiKeyAuthenticationHandler(ICacheService cacheService, IApiKeyService apiKeyService,
        IOptionsMonitor<ApiKeyAuthOptions> options,
        ILoggerFactory logger,
        UrlEncoder encoder, ISystemClock clock) : base(options, logger, encoder, clock)
    {
        _cacheService = cacheService;
        _apiKeyService = apiKeyService;
    }

    /// <inheritdoc />
    protected override async Task<AuthenticateResult> HandleAuthenticateAsync()
    {
        if (!Request.Headers.TryGetValue(ApiKeyAuthOptions.HeaderName, out var apiKey) || apiKey.Count != 1)
        {
            Logger.LogWarning("An API request was received without the x-api-key header");
            return AuthenticateResult.Fail("Invalid parameters");
        }

        var key = apiKey[0];
        if (!_apiKeyService.ValidateApiKeyFormat(key))
        {
            Logger.LogWarning($"An API request was received with an invalid API key: {apiKey}");
            return AuthenticateResult.Fail("Invalid parameters");
        }

        var clientId = await _cacheService.GetClientIdFromApiKey(key);

        if (clientId == Guid.Empty)
        {
            Logger.LogWarning($"An API request was received with an unknown API key: {apiKey}");
            return AuthenticateResult.Fail("Invalid parameters");
        }

        Logger.BeginScope("{ClientId}", clientId);
        Logger.LogInformation("Client authenticated");

        var claims = new[] { new Claim(ClaimTypes.Name, clientId.ToString()) };
        var identity = new ClaimsIdentity(claims, ApiKeyAuthOptions.DefaultScheme);
        var identities = new List<ClaimsIdentity> { identity };
        var principal = new ClaimsPrincipal(identities);
        var ticket = new AuthenticationTicket(principal, ApiKeyAuthOptions.DefaultScheme);

        return AuthenticateResult.Success(ticket);
    }
}