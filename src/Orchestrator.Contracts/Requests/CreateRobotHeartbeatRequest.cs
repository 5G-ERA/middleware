namespace Middleware.Orchestrator.Contracts.Requests;

public class CreateRobotHeartbeatRequest
{
    public Guid Id { get; set; }
    
    public Guid? ActionSequenceId { get; set; }

    public int? CurrentlyExecutedActionIndex { get; set; }

    public int BatteryLevel { get; set; }

    public double CpuUtilisation { get; set; }

    public double RamUtilisation { get; set; }
}