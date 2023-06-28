using FluentValidation;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Validation;

public class RobotRequestValidator : AbstractValidator<RobotRequest>
{
    public RobotRequestValidator()
    {
        var distroNames = RosDistroHelper.GetRosDistroNames();
        RuleFor(x => x.Name)
            .NotNull().NotEmpty();

        RuleFor(x => x.LocomotionSystem)
            .NotNull().IsEnumName(typeof(RobotLocomotionSystem), false)
            .WithMessage(x => $"{x.LocomotionSystem} is not a valid Locomotion system type name. " +
                              $"Valid options are: {string.Join(", ", Enum.GetNames<RobotLocomotionSystem>())}");
        RuleFor(x => x.LocomotionType)
            .NotNull().IsEnumName(typeof(RobotLocomotionType), false)
            .WithMessage(x => $"{x.LocomotionType} is not a valid Locomotion type name. " +
                              $"Valid options are: {string.Join(", ", Enum.GetNames<RobotLocomotionType>())}");

        RuleFor(x => x.RosVersion)
            .InclusiveBetween(1, 2);
        RuleFor(x => x.RosDistro)
            .NotNull().Custom((value, context) =>
            {
                if (!distroNames.Contains(value))
                {
                    context.AddFailure($"'{value}' is not a valid ROS distribution name" +
                                       $"Valid options are: {string.Join(", ", distroNames)}");
                }
            });

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