using Middleware.Models.Domain;
using Middleware.Orchestrator.Models;

namespace Middleware.Orchestrator.ExtensionMethods;

internal static class MappingExtensions
{
    public static RobotStatusModel ToRobotStatus(this RobotStatusRequest x)
    {
        return new RobotStatusModel()
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