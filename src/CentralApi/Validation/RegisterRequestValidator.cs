using FluentValidation;
using Middleware.CentralApi.Contracts.Requests;
using Middleware.Common.Enums;
using Middleware.Models.Enums;

namespace Middleware.CentralApi.Validation;

public class RegisterRequestValidator : AbstractValidator<RegisterRequest>
{
    public RegisterRequestValidator()
    {
        RuleFor(x => x.Name)
            .NotNull().NotEmpty();
        RuleFor(x => x.Organization)
            .NotNull().NotEmpty();
        RuleFor(x => x.Type)
            .NotNull().IsEnumName(typeof(LocationType), caseSensitive: false)
            .WithMessage(x => $"{x.Type} is not valid Location type." +
                              $"Valid options are: {string.Join(", ", Enum.GetNames<LocationType>())}");

    }
}