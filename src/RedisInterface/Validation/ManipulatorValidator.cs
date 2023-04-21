using FluentValidation;
using Middleware.Models.Domain;
using Middleware.Models.Enums;

namespace Middleware.RedisInterface.Validation;

public class ManipulatorValidator : AbstractValidator<ManipulatorModel>
{
    public ManipulatorValidator()
    {
        RuleFor(x => x.ActuatorName)
            .NotNull().NotEmpty();
        RuleFor(x => x.Number)
            .GreaterThan(0);
        RuleFor(x=>x.Type)
            .NotNull().IsEnumName(typeof(RobotActuatorType), caseSensitive: false)
            .WithMessage(x =>
                $"{x.Type} is not a valid actuator type name. " +
                $"Valid options are: {string.Join(", ", Enum.GetNames<RobotActuatorType>())}");
    }
}