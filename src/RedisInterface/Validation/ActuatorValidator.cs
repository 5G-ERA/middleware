using FluentValidation;
using Middleware.Models.Domain;
using Middleware.Models.Enums;

namespace Middleware.RedisInterface.Validation;

public class ActuatorValidator : AbstractValidator<ActuatorModel>
{
    public ActuatorValidator()
    {
        RuleFor(x => x.Name)
            .NotNull().NotEmpty();
        RuleFor(x => x.Type)
            .NotNull().IsEnumName(typeof(RobotActuatorType), caseSensitive: false)
            .WithMessage(x =>
                $"{x.Type} is not a valid actuator type name. " +
                $"Valid options are: {string.Join(", ", Enum.GetNames<RobotActuatorType>())}");
    }
}