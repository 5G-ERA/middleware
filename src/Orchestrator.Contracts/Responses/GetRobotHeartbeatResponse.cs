namespace Middleware.Orchestrator.Contracts.Responses;

public class GetRobotHeartbeatResponse
{
    public Guid Id { get; set; }

    public string? Name { get; set; }

    public Guid? ActionSequenceId { get; set; }

    public int? CurrentlyExecutedActionIndex { get; set; }

    public int BatteryLevel { get; set; }

    public double CpuUtilisation { get; set; }

    public double RamUtilisation { get; set; }

    public DateTimeOffset Timestamp { get; set; }
}