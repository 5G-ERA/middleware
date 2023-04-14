using FluentValidation;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Validation;

public class ActionRequestValidator : AbstractValidator<ActionRequest>
{
    public ActionRequestValidator()
    {
        RuleFor(x => x.Name)
            .NotNull()
            .NotEmpty();
        RuleFor(x => x.Priority)
            .NotEmpty().NotEqual("none", StringComparer.OrdinalIgnoreCase)
            .IsEnumName(typeof(Priority), caseSensitive: false)
            .WithMessage(x => $"{x.Priority} is not a valid option. " +
                              $"Valid options are {string.Join(", ", Enum.GetNames<Priority>().Where(p=>p != Priority.None.ToString()))}");
        RuleFor(x => x.Order)
            .NotNull()
            .GreaterThan(0);
        RuleFor(x => x.MinimumRam)
            .GreaterThan(0);
        RuleFor(x => x.MinimumNumCores)
            .GreaterThan(0);
    }
}