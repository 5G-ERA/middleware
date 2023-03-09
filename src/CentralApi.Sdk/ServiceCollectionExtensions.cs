using Microsoft.Extensions.DependencyInjection;
using Middleware.CentralApi.Sdk.Options;

namespace Middleware.CentralApi.Sdk;

public static class ServiceCollectionExtensions
{
    public static IServiceCollection AddCentralApiClient(this IServiceCollection services, string centralApiAddress, string organizationName)
    {
        if (centralApiAddress == null) throw new ArgumentNullException(nameof(centralApiAddress));
        if (organizationName == null) throw new ArgumentNullException(nameof(organizationName));

                
        services.Configure<LocationApiAccessOptions>(c =>
        {
            c.Organization = organizationName;
        });
        services.AddScoped<ICentralApiClient, CentralApiClient>();
        return services;
    }
}