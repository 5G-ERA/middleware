using Middleware.Common.Responses;
using Middleware.Models.Domain;

namespace Middleware.TaskPlanner.Config;

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
            // Orchestrator
            cfg.CreateMap<ActionModel, Orchestrator.ActionModel>().ReverseMap();
            cfg.CreateMap<InstanceModel, Orchestrator.InstanceModel>().ReverseMap();
            cfg.CreateMap<TaskModel, Orchestrator.TaskModel>().ReverseMap();
            cfg.CreateMap<ApiResponse, Orchestrator.ApiResponse>().ReverseMap();
            //cfg.CreateMap<ContainerImageModel, Orchestrator.ContainerImageModel>().ReverseMap();
            // Resource Planner
            cfg.CreateMap<TaskModel, ResourcePlanner.TaskModel>().ReverseMap();
            cfg.CreateMap<ApiResponse, ResourcePlanner.ApiResponse>().ReverseMap();
            cfg.CreateMap<ActionModel, ResourcePlanner.ActionModel>().ReverseMap();
            cfg.CreateMap<InstanceModel, ResourcePlanner.InstanceModel>().ReverseMap();
            cfg.CreateMap<RobotModel, ResourcePlanner.RobotModel>().ReverseMap();
            //cfg.CreateMap<ContainerImageModel, ResourcePlanner.ContainerImageModel>().ReverseMap();

        });
        return services;
    }
}