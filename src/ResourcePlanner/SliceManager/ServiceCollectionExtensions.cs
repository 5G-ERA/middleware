using Microsoft.Extensions.DependencyInjection;
using Refit;

namespace Middleware.ResourcePlanner.SliceManager;

public static class ServiceCollectionExtensions
{
    public static IServiceCollection AddSliceManager(this IServiceCollection services, string sliceManagerApiHostname)
    {
        if (sliceManagerApiHostname == null) throw new ArgumentNullException(nameof(sliceManagerApiHostname));

        services
            .AddRefitClient<ISliceManagerApi>()
            .ConfigureHttpClient(c => c.BaseAddress = new Uri($"http://{sliceManagerApiHostname}"));

        
        services.AddScoped<ISliceManager, SliceManager>();
        return services;
    }
}