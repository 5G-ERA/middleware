using FluentValidation;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Validation;

public class PolicyRequestValidator : AbstractValidator<PolicyRequest>
{
    public PolicyRequestValidator()
    {
        RuleFor(x => x.Name)
            .NotNull().NotEmpty();
        RuleFor(x => x.Description)
            .NotNull().NotEmpty();
        RuleFor(x=>x.Type)
            .NotNull().IsEnumName(typeof(PolicyType), caseSensitive: false)
            .WithMessage(x =>
                $"{x.Type} is not a validPolicy Type name. " +
                $"Valid options are: {string.Join(", ", Enum.GetNames<PolicyType>())}");
        //TODO: add validation if the policy can be active when it is added or activated
    }
}