using System.ComponentModel.DataAnnotations;

namespace Middleware.Models.Domain;

public class RobotStatusModel : BaseModel
{
    /// <summary>
    /// Identifier of the Robot
    /// </summary>
    [Required]
    public override Guid Id { get; set; }
    /// <summary>
    /// Name of the robot
    /// </summary>
    public override string Name { get; set; }

    /// <summary>
    /// Identifier of the currently executed action sequence
    /// </summary>
    [Required]
    public Guid ActionSequenceId { get; set; }
    /// <summary>
    /// Index of the currently executed action
    /// </summary>
    [Required]
    public int? CurrentlyExecutedActionIndex { get; set; }
    /// <summary>
    /// Battery level in %
    /// </summary>
    public int BatteryLevel { get; set; }
    /// <summary>
    /// Timestamp of the update
    /// </summary>
    public DateTimeOffset Timestamp { get; set; }

    public bool IsValid()
    {
        return Id != Guid.Empty && ActionSequenceId != Guid.Empty && CurrentlyExecutedActionIndex.HasValue;
    }
    public override object ToDto()
    {
        throw new NotImplementedException();
    }
}