using FluentValidation;
using Middleware.TaskPlanner.Contracts.Requests;

namespace Middleware.TaskPlanner.Validation;

public class CreatePlanRequestValidator : AbstractValidator<CreatePlanRequest>
{
    public CreatePlanRequestValidator()
    {
        RuleFor(x=>x.RobotId)
            .NotNull().NotEmpty();
        RuleFor(x => x.Id)
            .NotNull().NotEmpty();
    }
}