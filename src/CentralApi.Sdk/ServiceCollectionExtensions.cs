using Microsoft.Extensions.DependencyInjection;
using Middleware.CentralApi.Sdk.Client;
using Middleware.CentralApi.Sdk.Options;
using Refit;

namespace Middleware.CentralApi.Sdk;

public static class ServiceCollectionExtensions
{
    public static IServiceCollection AddCentralApiClient(this IServiceCollection services, string centralApiHostname,
        string organizationName)
    {
        if (centralApiHostname == null) throw new ArgumentNullException(nameof(centralApiHostname));
        if (organizationName == null) throw new ArgumentNullException(nameof(organizationName));

        services
            .AddRefitClient<ICentralApi>()
            .ConfigureHttpClient(c => c.BaseAddress = new Uri($"http://{centralApiHostname}"));

        services.Configure<LocationApiAccessOptions>(c => { c.Organization = organizationName; });
        services.AddScoped<ICentralApiClient, CentralApiClient>();
        return services;
    }
}