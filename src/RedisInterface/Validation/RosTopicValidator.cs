using FluentValidation;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Validation;

public class RosTopicValidator : AbstractValidator<RosTopicRequest>
{
    public RosTopicValidator()
    {
        RuleFor(x => x.Name).NotNull().NotEmpty()
            .Must(m => m.StartsWith('/'));
        RuleFor(x => x.Type).NotNull().NotEmpty();
    }
}