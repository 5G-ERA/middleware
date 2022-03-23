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
            cfg.CreateMap<ActionModel, RedisInterface.ActionModel>();
            cfg.CreateMap<CloudModel, RedisInterface.CloudModel>();
            cfg.CreateMap<ContainerImageModel, RedisInterface.ContainerImageModel>();
            cfg.CreateMap<DialogueModel, RedisInterface.DialogueModel>();
            cfg.CreateMap<EdgeModel, RedisInterface.EdgeModel>();
            cfg.CreateMap<InstanceModel, RedisInterface.InstanceModel>();
            cfg.CreateMap<KeyValuePair, RedisInterface.KeyValuePair>();
            cfg.CreateMap<PolicyModel, RedisInterface.PolicyModel>();
            cfg.CreateMap<RobotModel, RedisInterface.RobotModel>();
            cfg.CreateMap<TaskModel, RedisInterface.TaskModel>();
            cfg.CreateMap<PolicyModel, RedisInterface.PolicyModel>();

        });
        return services;
    }
}