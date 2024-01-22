using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Services;

internal class SystemConfigService : ISystemConfigService
{
    private readonly ISystemConfigRepository _systemConfigRepository;

    public SystemConfigService(ISystemConfigRepository systemConfigRepository)
    {
        _systemConfigRepository = systemConfigRepository;
    }

    /// <inheritdoc />
    public Task<SystemConfigModel> GetConfig()
    {
        return _systemConfigRepository.GetConfigAsync();
    }

    /// <inheritdoc />
    public async Task<(SystemConfigModel, string)> UpdateConfig(SystemConfigModel config)
    {
        var canUpdate = false;
        var reason = string.Empty;
        var defaultCfg = await _systemConfigRepository.GetConfigAsync();

        if (defaultCfg is null) return (null, "Did not find the config entry");

        if (config.HeartbeatExpirationInMinutes != default)
        {
            canUpdate = true;
            defaultCfg.HeartbeatExpirationInMinutes = config.HeartbeatExpirationInMinutes;
        }

        if (string.IsNullOrWhiteSpace(config.Ros1RelayContainer) == false)
        {
            canUpdate = true;
            defaultCfg.Ros1RelayContainer = config.Ros1RelayContainer;
        }

        if (string.IsNullOrWhiteSpace(config.Ros2RelayContainer) == false)
        {
            canUpdate = true;
            defaultCfg.Ros2RelayContainer = config.Ros2RelayContainer;
        }

        if (string.IsNullOrWhiteSpace(config.RosInterRelayNetAppContainer) == false)
        {
            canUpdate = true;
            defaultCfg.RosInterRelayNetAppContainer = config.RosInterRelayNetAppContainer;
        }

        if (canUpdate)
        {
            await _systemConfigRepository.InitializeConfigAsync(defaultCfg);
            return (defaultCfg, string.Empty);
        }

        return (null, reason);
    }
}