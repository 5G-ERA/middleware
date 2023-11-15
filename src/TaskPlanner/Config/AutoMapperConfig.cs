using Middleware.Common.Responses;
using Middleware.Models.Domain;
using KeyValuePair = Middleware.Models.Domain.KeyValuePair;

namespace Middleware.TaskPlanner.Config;

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
            cfg.CreateMap<DateTimeOffset, DateTime>().ConstructUsing(x => x.DateTime);
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
            //cfg.CreateMap<ManipulatorModel, ResourcePlanner.RobotManipulatorModel>().ReverseMap();
            cfg.CreateMap<DialogueModel, ResourcePlanner.DialogueModel>().ReverseMap();
            cfg.CreateMap<KeyValuePair, ResourcePlanner.KeyValuePair>().ReverseMap();
            cfg.CreateMap<RelationModel, ResourcePlanner.RelationModel>().ReverseMap();
            cfg.CreateMap<GraphEntityModel, ResourcePlanner.GraphEntityModel>().ReverseMap();
            cfg.CreateMap<ContainerImageModel, ResourcePlanner.ContainerImageModel>().ReverseMap();
        });
        return services;
    }
}