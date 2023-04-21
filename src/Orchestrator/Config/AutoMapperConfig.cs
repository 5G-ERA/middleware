namespace Middleware.Orchestrator.Config;

public static class AutoMapperConfig
{
    /// <summary>
    /// Configures AutoMapper for project and maps the models for other APIs
    /// </summary>
    /// <param name="services"></param>
    /// <returns></returns>
    public static IServiceCollection ConfigureAutoMapper(this IServiceCollection services)
    {
        services.AddAutoMapper(cfg =>
        {
            cfg.CreateMap<DateTimeOffset, DateTime>().ConstructUsing(x => x.DateTime);

           
        });
        return services;
    }
}