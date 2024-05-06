using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Common.Helpers;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.DataAccess.Repositories.Abstract.Influx;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Enums;

namespace Middleware.Orchestrator.Installer;

internal class StartupDataInstaller : IStartupDataInstaller
{
    private readonly IPolicyRepository _policyRepository;
    private readonly ISystemConfigRepository _systemConfigRepository;
    private readonly IOptions<UserConfig> _userConfig;
    private readonly IUserRepository _userRepository;
    private readonly IInfluxRobotStatusRepository _influxRobotStatusRepository;
    private readonly ILogger<StartupDataInstaller> _logger;

    public StartupDataInstaller(IUserRepository userRepository, IPolicyRepository policyRepository,
        ISystemConfigRepository systemConfigRepository,
        IOptions<UserConfig> userConfig,
        IInfluxRobotStatusRepository influxNetAppStatusRepository, ILogger<StartupDataInstaller> logger)
    {
        _userRepository = userRepository;
        _policyRepository = policyRepository;
        _systemConfigRepository = systemConfigRepository;
        _userConfig = userConfig;
        _influxRobotStatusRepository = influxNetAppStatusRepository;
        _logger = logger;
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

        if (IsConfigCorrect(cfg) == false) await _systemConfigRepository.InitializeConfigAsync(cfg);

        var user = CreateDefaultUser(_userConfig.Value);
        var existing = await _userRepository.GetByIdAsync(user.Id);
        if (existing is null)
            await _userRepository.AddAsync(user);

        var urllcPolicy = CreateUrllcPolicy();
        var existingPolicy = await _policyRepository.GetByIdAsync(urllcPolicy.Id);
        if (existingPolicy is null)
            await _policyRepository.AddAsync(urllcPolicy);

        var resourcePolicy = CreateResourceSelectionPolicy();
        var existingResourcePolicy = await _policyRepository.GetByIdAsync(resourcePolicy.Id);
        if (existingResourcePolicy is null)
            await _policyRepository.AddAsync(resourcePolicy);

        await CheckBucketsExist();
    }

    private async Task CheckBucketsExist()
    {
        var netappBucketName = NetAppStatusDto.Bucket;
        try
        {
            var netappBucketExist = await _influxRobotStatusRepository.GetBucketByNameAsync(netappBucketName);
            if (netappBucketName != netappBucketExist)
            {
                await CreateBucket(netappBucketName);
            }
            var robotBucketName = RobotStatusDto.Bucket;
            var robotBucketExist = await _influxRobotStatusRepository.GetBucketByNameAsync(robotBucketName);
            if (robotBucketName != robotBucketExist)
            {
                await CreateBucket(robotBucketName);
            }
        }
        catch (Exception ex)
        {
            _logger.LogCritical(ex, "Error while checking buckets exist");
        }
    }
    private async Task CreateBucket(string bucketName)
    {
        await _influxRobotStatusRepository.AddBucketAsync(bucketName, 2592000);

    }

    private bool IsConfigCorrect(SystemConfigModel cfg)
    {
        var defaultCfg = InitializeSystemConfigModel();
        var isCorrect = true;

        if (string.IsNullOrWhiteSpace(cfg.Ros1RelayContainer))
        {
            isCorrect = false;
            cfg.Ros1RelayContainer = defaultCfg.Ros1RelayContainer;
        }

        if (string.IsNullOrWhiteSpace(cfg.Ros2RelayContainer))
        {
            isCorrect = false;
            cfg.Ros2RelayContainer = defaultCfg.Ros2RelayContainer;
        }

        if (string.IsNullOrWhiteSpace(cfg.RosInterRelayNetAppContainer))
        {
            isCorrect = false;
            cfg.RosInterRelayNetAppContainer = defaultCfg.RosInterRelayNetAppContainer;
        }

        if (cfg.HeartbeatExpirationInMinutes == default)
        {
            isCorrect = false;
            cfg.HeartbeatExpirationInMinutes = defaultCfg.HeartbeatExpirationInMinutes;
        }

        if (string.IsNullOrEmpty(cfg.HermesContainer))
        {
            isCorrect = false;
            cfg.HermesContainer = defaultCfg.HermesContainer;
        }

        if (string.IsNullOrEmpty(cfg.S3DataPersistenceRegion))
        {
            isCorrect = false;
            cfg.S3DataPersistenceRegion = defaultCfg.S3DataPersistenceRegion;
        }

        if (string.IsNullOrEmpty(cfg.S3DataPersistenceBucketName))
        {
            isCorrect = false;
            cfg.S3DataPersistenceBucketName = defaultCfg.S3DataPersistenceBucketName;
        }

        return isCorrect;
    }

    private SystemConfigModel InitializeSystemConfigModel()
    {
        var cfg = new SystemConfigModel
        {
            Ros1RelayContainer = "but5gera/relay_network_application:0.4.4",
            Ros2RelayContainer = "but5gera/ros2_relay_server:1.2.2",
            RosInterRelayNetAppContainer = "but5gera/inter_relay_network_application:0.4.4",
            HeartbeatExpirationInMinutes = 3,
            S3DataPersistenceBucketName = "middleware-netapps",
            S3DataPersistenceRegion = "eu-west-2",
            HermesContainer = "ghcr.io/5g-era/hermes:v1.0.0"
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

    private PolicyModel CreateResourceSelectionPolicy()
    {
        return new()
        {
            Id = Guid.Parse("AC10B7E7-8B71-4548-BD17-90AACCEFF270"),
            Name = "ResourceBasedLocation",
            Description = "Automatically adds resource-based location selection for the NetApps deployment",
            IsActive = true,
            IsExclusiveWithinType = 0,
            Priority = Priority.Normal,
            Scope = PolicyScope.System,
            Type = PolicyType.LocationSelection,
            Timestamp = DateTimeOffset.Now.DateTime
        };
    }


    private UserModel CreateDefaultUser(UserConfig config)
    {
        var hasValue = !string.IsNullOrWhiteSpace(config.Password) && !string.IsNullOrWhiteSpace(config.Username);
        var password = hasValue ? config.Password : "middleware";
        var name = hasValue ? config.Username : "middleware";
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