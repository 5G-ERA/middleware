using Middleware.Models.Domain;
using Middleware.Orchestrator.Contracts.Responses;

namespace Middleware.Orchestrator.Contracts.Mappings;

public static class DomainToApiContractMapper
{
    public static GetRobotHeartbeatResponse ToRobotHeartbeatResponse(this RobotStatusModel x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            ActionSequenceId = x.ActionSequenceId,
            CurrentlyExecutedActionIndex = x.CurrentlyExecutedActionIndex,
            CpuUtilisation = x.CpuUtilisation,
            RamUtilisation = x.RamUtilisation,
            BatteryLevel = x.BatteryLevel,
            Timestamp = x.Timestamp
        };
    }

    public static GetNetAppHeartbeatResponse ToNetAppHeartbeatResponse(this NetAppStatusModel x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            CurrentRobotsCount = x.CurrentRobotsCount,
            HardLimit = x.HardLimit,
            OptimalLimit = x.OptimalLimit,
            Colour = x.Colour,
            Timestamp = x.Timestamp
        };
    }

    public static GetRobotsHeartbeatResponse ToRobotsHeartbeatResponse(this IEnumerable<RobotStatusModel> x)
    {
        return new()
        {
            Robots = x.Select(t => t.ToRobotHeartbeatResponse())
        };
    }
    public static GetNetAppsHeartbeatResponse ToNetAppsHeartbeatResponse(this IEnumerable<NetAppStatusModel> x)
    {
        return new()
        {
            NetApps = x.Select(t => t.ToNetAppHeartbeatResponse())
        };
    }
}