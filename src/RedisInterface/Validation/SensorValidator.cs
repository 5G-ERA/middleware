using FluentValidation;
using Middleware.Models.Domain;
using Middleware.Models.Enums;

namespace Middleware.RedisInterface.Validation;

public class SensorValidator : AbstractValidator<SensorModel>
{
    public SensorValidator()
    {
        RuleFor(x => x.Name)
            .NotNull().NotEmpty();
        RuleFor(x => x.Description)
            .NotNull().NotEmpty();
        RuleFor(x => x.Number)
            .GreaterThan(0);
        RuleFor(x=>x.Type)
            .NotNull().IsEnumName(typeof(SensorType), caseSensitive: false)
            .WithMessage(x =>
                $"{x.Type} is not a valid sensor type name. " +
                $"Valid options are: {string.Join(", ", Enum.GetNames<SensorType>())}");
        
    }
}