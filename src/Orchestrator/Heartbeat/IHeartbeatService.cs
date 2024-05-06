using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Heartbeat;

public interface IHeartbeatService
{
    Task<NetAppStatusModel> GetNetAppStatusByIdAsync(Guid id, bool generateFakeData = false);
    Task<RobotStatusModel> GetRobotStatusByIdAsync(Guid id, bool generateFakeData = false);
    Task<IReadOnlyList<NetAppStatusModel>> GetAllAppStatusesAsync(bool generateFakeData = false);
    Task<IReadOnlyList<RobotStatusModel>> GetAllRobotStatusesAsync(bool generateFakeData = false);

    Task<HeartbeatResult<RobotStatusModel>> AddRobotStatus(RobotStatusModel status);

    Task<HeartbeatResult<NetAppStatusModel>> AddNetAppStatus(NetAppStatusModel status);
}