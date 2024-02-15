using Middleware.Models.Domain;

using Middleware.Models.Enums;
using Middleware.Orchestrator.Contracts.Requests;

namespace Middleware.Orchestrator.Contracts.Mappings;

public static class ApiContractToDomainMapper
{
    public static NetAppStatusModel ToNetAppStatus(this CreateNetAppHeartbeatRequest x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name!,
            HardLimit = x.HardLimit,
            OptimalLimit = x.OptimalLimit,
            CurrentRobotsCount = x.CurrentRobotsCount
        };
    }
    public static RobotStatusModel ToRobotStatus(this CreateRobotHeartbeatRequest x)
    {
        return new()
        {
            Id = x.Id,
            BatteryLevel = x.BatteryLevel,
            ActionSequenceId = x.ActionSequenceId,
            CurrentlyExecutedActionIndex = x.CurrentlyExecutedActionIndex,
            CpuUtilisation = x.CpuUtilisation,
            RamUtilisation = x.RamUtilisation
        };
    }
}