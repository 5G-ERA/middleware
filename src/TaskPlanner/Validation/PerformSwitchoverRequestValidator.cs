using FluentValidation;
using Middleware.TaskPlanner.Contracts.Requests;

namespace Middleware.TaskPlanner.Validation;

public class PerformSwitchoverRequestValidator : AbstractValidator<PerformSwitchoverRequest>
{
    public PerformSwitchoverRequestValidator()
    {
        RuleFor(x => x.ActionId)
            .NotNull().NotEmpty();
        RuleFor(x => x.ActionPlanId)
            .NotNull().NotEmpty();

        RuleFor(x => x.Destination)
            .NotNull().NotEmpty();
    }
}