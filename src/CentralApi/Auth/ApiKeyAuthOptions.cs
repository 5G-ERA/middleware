using Microsoft.AspNetCore.Authentication;

namespace Middleware.CentralApi.Auth;

internal class ApiKeyAuthOptions : AuthenticationSchemeOptions
{
    public const string DefaultScheme = "ClientKey";
    public const string HeaderName = "x-api-key";
}