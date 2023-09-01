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
                $"{x.Type} is not a valid Policy Type name. " +
                $"Valid options are: {string.Join(", ", Enum.GetNames<PolicyType>())}");

        RuleFor(x => x.Scope)
            .NotNull().IsEnumName(typeof(PolicyScope), caseSensitive: false)
            .WithMessage(x =>
                $"{x.Scope} is not a valid Policy Scope name. " +
                $"Valid options are: {string.Join(", ", Enum.GetNames<PolicyScope>())}");

        RuleFor(x => x.Priority)
            .NotNull().IsEnumName(typeof(Priority), caseSensitive: false)
            .WithMessage(x =>
                $"{x.Priority} is not a valid Policy Priority name. " +
                $"Valid options are: {string.Join(", ", Enum.GetNames<Priority>())}");
        //TODO: add validation if the policy can be active when it is added or activated
    }
}