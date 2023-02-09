using Middleware.Common.Models;
using Middleware.Common.Responses;


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

            // Resource Planner
            cfg.CreateMap<TaskModel, ResourcePlanner.TaskModel>().ReverseMap();
            cfg.CreateMap<ApiResponse, ResourcePlanner.ApiResponse>().ReverseMap();
            cfg.CreateMap<ActionModel, ResourcePlanner.ActionModel>().ReverseMap();
            cfg.CreateMap<InstanceModel, ResourcePlanner.InstanceModel>().ReverseMap();
            cfg.CreateMap<RobotModel, ResourcePlanner.RobotModel>().ReverseMap();
            
            cfg.CreateMap<ROSNodeModel, ResourcePlanner.ROSNodeModel>().ReverseMap();
            cfg.CreateMap<RosTopicModel, ResourcePlanner.RosTopicModel>().ReverseMap();
            cfg.CreateMap<ROSServiceModel, ResourcePlanner.ROSServiceModel>().ReverseMap();
            cfg.CreateMap<SensorModel, ResourcePlanner.SensorModel>().ReverseMap();
            cfg.CreateMap<ActuatorModel, ResourcePlanner.ActuatorModel>().ReverseMap();
            cfg.CreateMap<RobotManipulatorModel, ResourcePlanner.RobotManipulatorModel>().ReverseMap();
            cfg.CreateMap<DialogueModel, ResourcePlanner.DialogueModel>().ReverseMap();
            cfg.CreateMap<Common.Models.KeyValuePair, ResourcePlanner.KeyValuePair>().ReverseMap();
            cfg.CreateMap<RelationModel, ResourcePlanner.RelationModel>().ReverseMap();
            cfg.CreateMap<GraphEntityModel, ResourcePlanner.GraphEntityModel>().ReverseMap();

        });
        return services;
    }
}