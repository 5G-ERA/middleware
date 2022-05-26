namespace Middleware.Common.Models;

public class RobotStatusModel : BaseModel
{
    /// <summary>
    /// Identifier of the Robot
    /// </summary>
    public override Guid Id { get; set; }
    /// <summary>
    /// Name of the robot
    /// </summary>
    public override string Name { get; set; }
    /// <summary>
    /// Identifier of the currently executed action sequence
    /// </summary>
    public Guid ActionSequenceId { get; set; }
    /// <summary>
    /// Index of the currently executed action
    /// </summary>
    public int CurrentlyExecutedActionIndex { get; set; }
    /// <summary>
    /// Battery level in %
    /// </summary>
    public int BatteryLevel { get; set; }
}