using Middleware.Models.Domain;

namespace Middleware.RedisInterface.Services.Abstract;

public interface ISystemConfigService
{
    Task<SystemConfigModel> GetConfig();
    Task<(SystemConfigModel, string)> UpdateConfig(SystemConfigModel config);
}