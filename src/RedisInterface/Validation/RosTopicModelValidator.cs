using FluentValidation;
using Middleware.Models.Domain;

namespace Middleware.RedisInterface.Validation;

public class RosTopicModelValidator : AbstractValidator<RosTopicModel>
{
    public RosTopicModelValidator()
    {
        RuleFor(x => x.Name).NotNull().NotEmpty();
        RuleFor(x => x.Type).NotNull().NotEmpty();
    }
}