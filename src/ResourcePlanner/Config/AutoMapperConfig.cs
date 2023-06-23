using Middleware.Common.Responses;
using Middleware.Models.Domain;

namespace Middleware.ResourcePlanner.Config;

public static class AutoMapperConfig
{
    /// <summary>
    ///     Configures AutoMapper for project and maps the models for other APIs
    /// </summary>
    /// <param name="services"></param>
    /// <returns></returns>
    public static IServiceCollection ConfigureAutoMapper(this IServiceCollection services)
    {
        services.AddAutoMapper(cfg =>
        {
            // Orchestrator
            cfg.CreateMap<ActionModel, Orchestrator.ActionModel>().ReverseMap();
            cfg.CreateMap<InstanceModel, Orchestrator.InstanceModel>().ReverseMap();
            cfg.CreateMap<TaskModel, Orchestrator.TaskModel>().ReverseMap();
            cfg.CreateMap<ApiResponse, Orchestrator.ApiResponse>().ReverseMap();
        });
        return services;
    }
}