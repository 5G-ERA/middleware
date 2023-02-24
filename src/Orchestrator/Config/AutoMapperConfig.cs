using Middleware.Common.Responses;
using Middleware.Models.Domain;
using KeyValuePair = Middleware.Models.Domain.KeyValuePair;

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
            cfg.CreateMap<ActionPlanModel, RedisInterface.ActionPlanModel>().ReverseMap();
            cfg.CreateMap<ApiResponse, RedisInterface.ApiResponse>().ReverseMap();

            cfg.CreateMap<ROSNodeModel, RedisInterface.ROSNodeModel>().ReverseMap();
            cfg.CreateMap<RosTopicModel, RedisInterface.RosTopicModel>().ReverseMap();
            cfg.CreateMap<ROSServiceModel, RedisInterface.ROSServiceModel>().ReverseMap();
            cfg.CreateMap<SensorModel, RedisInterface.SensorModel>().ReverseMap();
            cfg.CreateMap<ActuatorModel, RedisInterface.ActuatorModel>().ReverseMap();
            //cfg.CreateMap<ManipulatorModel, RedisInterface.>().ReverseMap();
            cfg.CreateMap<DialogueModel, RedisInterface.DialogueModel>().ReverseMap();
            cfg.CreateMap<KeyValuePair, RedisInterface.KeyValuePair>().ReverseMap();
            cfg.CreateMap<RelationModel, RedisInterface.RelationModel>().ReverseMap();
            cfg.CreateMap<GraphEntityModel, RedisInterface.GraphEntityModel>().ReverseMap();
        });
        return services;
    }
}