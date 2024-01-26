using Middleware.Models.Domain.Ros;
using Middleware.TaskPlanner.ResourcePlanner;
using ActionModel = Middleware.Models.Domain.ActionModel;
using ActuatorModel = Middleware.Models.Domain.ActuatorModel;
using ApiResponse = Middleware.Common.Responses.ApiResponse;
using ContainerImageModel = Middleware.Models.Domain.ContainerImageModel;
using DialogueModel = Middleware.Models.Domain.DialogueModel;
using GraphEntityModel = Middleware.Models.Domain.GraphEntityModel;
using InstanceModel = Middleware.Models.Domain.InstanceModel;
using KeyValuePair = Middleware.Models.Domain.KeyValuePair;
using RelationModel = Middleware.Models.Domain.RelationModel;
using RobotModel = Middleware.Models.Domain.RobotModel;
using RosTopicModel = Middleware.Models.Domain.Ros.RosTopicModel;
using SensorModel = Middleware.Models.Domain.SensorModel;
using TaskModel = Middleware.Models.Domain.TaskModel;

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

            cfg.CreateMap<RosNodeModel, ROSNodeModel>().ReverseMap();
            cfg.CreateMap<RosTopicModel, ResourcePlanner.RosTopicModel>().ReverseMap();
            cfg.CreateMap<RosServiceModel, ROSServiceModel>().ReverseMap();
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