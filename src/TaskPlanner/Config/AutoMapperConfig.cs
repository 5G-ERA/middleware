using Middleware.Common.Models;
using KeyValuePair = Middleware.Common.Models.KeyValuePair;

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
            //Redis Interface
            cfg.CreateMap<ActionModel, RedisInterface.ActionModel>().ReverseMap();
            cfg.CreateMap<CloudModel, RedisInterface.CloudModel>().ReverseMap();
            cfg.CreateMap<ContainerImageModel, RedisInterface.ContainerImageModel>().ReverseMap();
            cfg.CreateMap<DialogueModel, RedisInterface.DialogueModel>().ReverseMap();
            cfg.CreateMap<EdgeModel, RedisInterface.EdgeModel>().ReverseMap();
            cfg.CreateMap<InstanceModel, RedisInterface.InstanceModel>().ReverseMap();
            cfg.CreateMap<KeyValuePair, RedisInterface.KeyValuePair>().ReverseMap();
            cfg.CreateMap<PolicyModel, RedisInterface.PolicyModel>().ReverseMap();
            cfg.CreateMap<RobotModel, RedisInterface.RobotModel>().ReverseMap();
            cfg.CreateMap<TaskModel, RedisInterface.TaskModel>().ReverseMap();
            cfg.CreateMap<PolicyModel, RedisInterface.PolicyModel>().ReverseMap();
            cfg.CreateMap<RelationModel, RedisInterface.RelationModel>().ReverseMap();
            cfg.CreateMap<GraphEntityModel, RedisInterface.GraphEntityModel>().ReverseMap();
            cfg.CreateMap<ApiResponse, RedisInterface.ApiResponse>().ReverseMap();
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
            //cfg.CreateMap<ContainerImageModel, ResourcePlanner.ContainerImageModel>().ReverseMap();

        });
        return services;
    }
}