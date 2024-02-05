using System.ComponentModel.DataAnnotations;
using Middleware.Models.Dto;

namespace Middleware.Models.Domain;

public class RobotStatusModel : BaseModel
{
    /// <summary>
    ///     Identifier of the Robot
    /// </summary>
    [Required]
    public override Guid Id { get; set; }

    /// <summary>
    ///     Name of the robot
    /// </summary>
    public override string Name { get; set; } = default!;

    /// <summary>
    ///     Identifier of the currently executed action sequence
    /// </summary>
    public Guid? ActionSequenceId { get; set; }

    /// <summary>
    ///     Index of the currently executed action
    /// </summary>
    public int? CurrentlyExecutedActionIndex { get; set; }

    /// <summary>
    ///     Battery level in %
    /// </summary>
    public int BatteryLevel { get; set; }

    /// <summary>
    ///  Current CPU utilisation of the robot
    /// </summary>
    public double CpuUtilisation { get; set; }

    /// <summary>
    ///  Current RAM utilisation of the robot
    /// </summary>
    public double RamUtilisation { get; set; }

    /// <summary>
    ///     Timestamp of the update
    /// </summary>
    public DateTimeOffset Timestamp { get; set; }

    public bool IsValid()
    {
        return Id != Guid.Empty && ActionSequenceId != Guid.Empty && CurrentlyExecutedActionIndex.HasValue;
    }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new RobotStatusDto
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            ActionSequenceId = domain.ActionSequenceId?.ToString(),
            CurrentlyExecutedActionIndex = domain.CurrentlyExecutedActionIndex,
            BatteryLevel = domain.BatteryLevel,
            CpuUtilisation = domain.CpuUtilisation,
            RamUtilisation = domain.RamUtilisation,
            Timestamp = domain.Timestamp
        };
    }
}