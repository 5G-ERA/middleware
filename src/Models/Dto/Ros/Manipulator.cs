using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Ros;

public class Manipulator
{
    /// <summary>
    /// Actuator Name
    /// </summary>
    [Indexed]
    public string? ActuatorName { get; set; }
    /// <summary>
    /// Degrees of freedom
    /// </summary>
    [Indexed]
    public int Dof { get; set; }

    [Indexed]
    public int Number { get; set; }


    public ManipulatorModel ToModel()
    {
        var dto = this;
        return new ManipulatorModel()
        {
            ActuatorName = dto.ActuatorName,
            Dof = dto.Dof,
            Number = dto.Number
        };
    }
}