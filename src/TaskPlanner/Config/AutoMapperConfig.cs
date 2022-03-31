using AutoMapper;
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
            //Redis Interface
            cfg.CreateMap<ActionModel, RedisInterface.ActionModel>();
            cfg.CreateMap<RedisInterface.ActionModel, ActionModel>();
            cfg.CreateMap<CloudModel, RedisInterface.CloudModel>();
            cfg.CreateMap<RedisInterface.CloudModel, CloudModel>();
            cfg.CreateMap<ContainerImageModel, RedisInterface.ContainerImageModel>();
            cfg.CreateMap<RedisInterface.ContainerImageModel, ContainerImageModel>();
            cfg.CreateMap<DialogueModel, RedisInterface.DialogueModel>();
            cfg.CreateMap<RedisInterface.DialogueModel, DialogueModel>();
            cfg.CreateMap<EdgeModel, RedisInterface.EdgeModel>();
            cfg.CreateMap<RedisInterface.EdgeModel, EdgeModel>();
            cfg.CreateMap<InstanceModel, RedisInterface.InstanceModel>();
            cfg.CreateMap<RedisInterface.InstanceModel, InstanceModel>();
            cfg.CreateMap<KeyValuePair, RedisInterface.KeyValuePair>();
            cfg.CreateMap<RedisInterface.KeyValuePair, KeyValuePair>();
            cfg.CreateMap<PolicyModel, RedisInterface.PolicyModel>();
            cfg.CreateMap<RedisInterface.PolicyModel, PolicyModel>();
            cfg.CreateMap<RobotModel, RedisInterface.RobotModel>();
            cfg.CreateMap<RedisInterface.RobotModel, RobotModel>();
            cfg.CreateMap<TaskModel, RedisInterface.TaskModel>();
            cfg.CreateMap<RedisInterface.TaskModel, TaskModel>();
            cfg.CreateMap<PolicyModel, RedisInterface.PolicyModel>();
            cfg.CreateMap<RedisInterface.PolicyModel, PolicyModel>();
            cfg.CreateMap<RelationModel, RedisInterface.RelationModel>();
            cfg.CreateMap<RedisInterface.RelationModel, RelationModel>();
            cfg.CreateMap<GraphEntityModel, RedisInterface.GraphEntityModel>();
            cfg.CreateMap<RedisInterface.GraphEntityModel, GraphEntityModel>();
            
            // Orchestrator
            cfg.CreateMap<ActionModel, Orchestrator.ActionModel>();
            cfg.CreateMap<Orchestrator.ActionModel, ActionModel>();
            cfg.CreateMap<InstanceModel, Orchestrator.InstanceModel>();
            cfg.CreateMap<Orchestrator.InstanceModel, InstanceModel>();
            cfg.CreateMap<TaskModel, Orchestrator.TaskModel>();
            cfg.CreateMap<Orchestrator.TaskModel, TaskModel>();

            // Resource Planner
            cfg.CreateMap<TaskModel, ResourcePlanner.TaskModel>();
            cfg.CreateMap<ResourcePlanner.TaskModel, TaskModel>();
            cfg.CreateMap<ActionModel, ResourcePlanner.ActionModel>();
            cfg.CreateMap<ResourcePlanner.ActionModel, ActionModel>();
            cfg.CreateMap<InstanceModel, ResourcePlanner.InstanceModel>();
            cfg.CreateMap<ResourcePlanner.InstanceModel, InstanceModel>();

        });
        return services;
    }
}