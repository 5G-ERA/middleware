using Middleware.DataAccess.Repositories.Abstract;
using Middleware.DataAccess.Repositories.Abstract.Influx;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Sdk;

namespace Middleware.Orchestrator.Heartbeat;

internal class HeartbeatService : IHeartbeatService
{
    private readonly ILogger _logger;
    private readonly INetAppStatusRepository _netAppStatusRepository;
    private readonly IRedisInterfaceClient _redisInterfaceClient;
    private readonly IRobotStatusRepository _robotStatusRepository;

    private readonly IInfluxNetAppStatusRepository _influxNetAppStatusRepository;
    private readonly IInfluxRobotStatusRepository _influxRobotStatusRepository;

    public HeartbeatService(IRobotStatusRepository robotStatusRepository, IRedisInterfaceClient redisInterfaceClient,
        INetAppStatusRepository netAppStatusRepository, ILogger<HeartbeatService> logger,
        IInfluxNetAppStatusRepository influxNetAppStatusRepository,
        IInfluxRobotStatusRepository influxRobotStatusRepository)
    {
        _robotStatusRepository = robotStatusRepository;
        _redisInterfaceClient = redisInterfaceClient;
        _netAppStatusRepository = netAppStatusRepository;
        _logger = logger;
        _influxNetAppStatusRepository = influxNetAppStatusRepository;
        _influxRobotStatusRepository = influxRobotStatusRepository;
    }

    /// <inheritdoc />
    public async Task<NetAppStatusModel> GetNetAppStatusByIdAsync(Guid id, bool generateFakeData = false)
    {
        var status = await _netAppStatusRepository.GetByIdAsync(id);
        if (status is null && generateFakeData)
        {
            var robotResp = await _redisInterfaceClient.InstanceGetByIdAsync(id);
            if (robotResp is null) return null;
            return CreateNewAppStatus(id, robotResp.Name);
        }

        return status;
    }

    /// <inheritdoc />
    public async Task<RobotStatusModel> GetRobotStatusByIdAsync(Guid id, bool generateFakeData = false)
    {
        var status = await _robotStatusRepository.GetByIdAsync(id);
        if (status is null && generateFakeData)
        {
            var robotResp = await _redisInterfaceClient.RobotGetByIdAsync(id);
            if (robotResp is null) return null;
            return CreateNewRobotStatus(id, robotResp.Name);
        }

        return status;
    }

    /// <inheritdoc />
    public async Task<IReadOnlyList<NetAppStatusModel>> GetAllAppStatusesAsync(bool generateFakeData)
    {
        var statuses = await _netAppStatusRepository.GetAllAsync();
        if (!statuses.Any() && generateFakeData) return await CreateFakeNetAppStatusList();
        return statuses;
    }

    /// <inheritdoc />
    public async Task<IReadOnlyList<RobotStatusModel>> GetAllRobotStatusesAsync(bool generateFakeData = false)
    {
        var statuses = await _robotStatusRepository.GetAllAsync();
        if (!statuses.Any() && generateFakeData) return await CreateFakeRobotStatusList();
        return statuses;
    }

    private async Task<IReadOnlyList<RobotStatusModel>> CreateFakeRobotStatusList()
    {
        var robotsResponse = await _redisInterfaceClient.RobotGetAllAsync();
        if (robotsResponse is null) return null;

        return robotsResponse.Robots.Select(i => CreateNewRobotStatus(i.Id, i.Name)).ToList();
    }

    private async Task<IReadOnlyList<NetAppStatusModel>> CreateFakeNetAppStatusList()
    {
        var instancesResp = await _redisInterfaceClient.InstanceGetAllAsync();
        if (instancesResp is null) return null;

        return instancesResp.Instances.Select(i => CreateNewAppStatus(i.Id, i.Name)).ToList();
    }

    private NetAppStatusModel CreateNewAppStatus(Guid id, string name)
    {
        return new()
        {
            Id = id,
            Name = name,
            Timestamp = DateTimeOffset.Now - TimeSpan.FromSeconds(Random.Shared.Next(1, 10)),
            CurrentRobotsCount = Random.Shared.Next(0, 3),
            OptimalLimit = Random.Shared.Next(2, 4),
            HardLimit = Random.Shared.Next(4, 6)
        };
    }

    private RobotStatusModel CreateNewRobotStatus(Guid id, string name)
    {
        return new()
        {
            Id = id,
            Name = name,
            Timestamp = DateTimeOffset.Now - TimeSpan.FromSeconds(Random.Shared.Next(1, 10)),
            ActionSequenceId = Guid.NewGuid(),
            CurrentlyExecutedActionIndex = Random.Shared.Next(1, 4),
            BatteryLevel = Random.Shared.Next(1, 100)
        };
    }
}