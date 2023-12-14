using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain;

public class ManipulatorModel
{
    public string ActuatorName { get; set; } = default!;

    /// <summary>
    ///     Degree's of freedom
    /// </summary>
    public int Dof { get; set; }

    public int Number { get; set; }
    public string Type { get; set; } = default!;


    public Manipulator ToDto()
    {
        return new() //
        {
            ActuatorName = ActuatorName,
            Dof = Dof,
            Number = Number
        };
    }
}