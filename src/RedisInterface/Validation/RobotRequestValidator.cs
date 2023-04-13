using FluentValidation;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Validation;

public class RobotRequestValidator : AbstractValidator<RobotRequest>
{
    public RobotRequestValidator()
    {
        RuleFor(x => x.Name)
            .NotNull().NotEmpty();

        RuleFor(x => x.LocomotionSystem)
            .NotNull().IsEnumName(typeof(RobotLocomotionSystem), caseSensitive: false)
            .WithMessage(x => $"{x.LocomotionSystem} is not a valid Locomotion system type name. " +
                              $"Valid options are: {string.Join(", ", Enum.GetNames<RobotLocomotionSystem>())}");
        RuleFor(x => x.LocomotionType)
            .NotNull().IsEnumName(typeof(RobotLocomotionType), caseSensitive: false)
            .WithMessage(x => $"{x.LocomotionType} is not a valid Locomotion type name. " +
                              $"Valid options are: {string.Join(", ", Enum.GetNames<RobotLocomotionType>())}");

        RuleFor(x => x.RosVersion)
            .InclusiveBetween(1, 2);
        RuleFor(x => x.RosDistro)
            .NotNull().IsEnumName(typeof(RosDistro), caseSensitive: false)
            .WithMessage(x =>
                $"{x.RosDistro} is not a valid ROS Distribution name. " +
                $"Valid options are: {string.Join(", ", Enum.GetNames<RosDistro>())}");
        
        RuleForEach(x => x.Sensors)
            .SetValidator(new SensorValidator());
        RuleForEach(x => x.Actuators)
            .SetValidator(new ActuatorValidator());
        RuleForEach(x => x.Manipulators)
            .SetValidator(new ManipulatorValidator());
        
        RuleFor(x => x.Cpu)
            .GreaterThan(0);
        RuleFor(x => x.NumberOfCores)
            .GreaterThan(0);
        RuleFor(x => x.Ram)
            .GreaterThan(0);
        RuleFor(x => x.StorageDisk)
            .GreaterThan(0);
    }
}