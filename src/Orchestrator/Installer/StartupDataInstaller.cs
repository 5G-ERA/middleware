using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Common.Helpers;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Enums;

namespace Middleware.Orchestrator.Installer;

internal class StartupDataInstaller : IStartupDataInstaller
{
    private readonly IPolicyRepository _policyRepository;
    private readonly ISystemConfigRepository _systemConfigRepository;
    private readonly IOptions<UserConfig> _userConfig;
    private readonly IUserRepository _userRepository;

    public StartupDataInstaller(IUserRepository userRepository, IPolicyRepository policyRepository,
        ISystemConfigRepository systemConfigRepository,
        IOptions<UserConfig> userConfig)
    {
        _userRepository = userRepository;
        _policyRepository = policyRepository;
        _systemConfigRepository = systemConfigRepository;
        _userConfig = userConfig;
    }

    /// <inheritdoc />
    public async Task InitializeStartupDataAsync()
    {
        var cfg = await _systemConfigRepository.GetConfigAsync();
        if (cfg is null)
        {
            cfg = InitializeSystemConfigModel();
            await _systemConfigRepository.InitializeConfigAsync(cfg);
        }

        var user = CreateDefaultUser(_userConfig.Value);
        var existing = await _userRepository.GetByIdAsync(user.Id);
        if (existing is null)
            await _userRepository.AddAsync(user);

        var urllcPolicy = CreateUrllcPolicy();
        var existingPolicy = await _policyRepository.GetByIdAsync(urllcPolicy.Id);
        if (existingPolicy is null)
            await _policyRepository.AddAsync(urllcPolicy);
    }

    private SystemConfigModel InitializeSystemConfigModel()
    {
        var cfg = new SystemConfigModel
        {
            Ros1RelayContainer = "but5gera/relay_network_application:0.4.4",
            Ros2RelayContainer = "but5gera/ros2_relay_server:0.1.0",
            RosInterRelayNetAppContainer = "but5gera/inter_relay_network_application:0.4.4"
        };


        return cfg;
    }

    private PolicyModel CreateUrllcPolicy()
    {
        return new()
        {
            Id = Guid.Parse("70837862-0f57-403d-b1f3-5670c313b67a"),
            Name = "UrllcSliceLocation",
            Description = "Selects the location that has URLLC slices enabled",
            IsActive = true,
            IsExclusiveWithinType = 0,
            Priority = Priority.None,
            Scope = PolicyScope.Resource,
            Type = PolicyType.LocationSelection,
            Timestamp = DateTimeOffset.Now.DateTime
        };
    }

    private UserModel CreateDefaultUser(UserConfig config)
    {
        var isEmpty = !string.IsNullOrWhiteSpace(config.Password) && !string.IsNullOrWhiteSpace(config.Username);
        var password = isEmpty ? "middleware" : config.Password;
        var name = isEmpty ? "middleware" : config.Username;
        var salt = AuthHelper.GetSalt();

        return new()
        {
            Id = Guid.Parse("AD20F254-DC3B-406D-9F15-B73CCD47E867"),
            Name = name,
            Password = AuthHelper.HashPasswordWithSalt(password, salt),
            Role = "admin",
            Salt = AuthHelper.StringifySalt(salt)
        };
    }
}