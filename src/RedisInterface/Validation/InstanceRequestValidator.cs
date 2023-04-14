using FluentValidation;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Validation;

public class InstanceRequestValidator : AbstractValidator<InstanceRequest>
{
    public InstanceRequestValidator()
    {
        RuleFor(x => x.Name)
            .NotNull().NotEmpty();
        RuleFor(x => x.Type)
            .NotNull().NotEmpty();
        RuleFor(x => x.IsReusable)
            .NotNull();
        RuleForEach(x => x.RosTopicPublishers)
            .NotNull().NotEmpty().SetValidator(new RosTopicModelValidator());
        RuleForEach(x => x.RosTopicSubscribers)
            .NotNull().NotEmpty().SetValidator(new RosTopicModelValidator());
        RuleFor(x => x.RosVersion)
            .NotNull().InclusiveBetween(1, 2);
        RuleFor(x => x.RosDistro)
            .NotNull().NotEmpty()
            .IsEnumName(typeof(RosDistro), caseSensitive: false)
            .WithMessage(x =>
                $"{x.RosDistro} is not a valid ROS Distribution name. " +
                $"Valid options are: {string.Join(", ", Enum.GetNames<RosDistro>())}");
        RuleFor(x => x.Family)
            .NotNull().NotEmpty();
        RuleFor(x => x.MinimumRam)
            .GreaterThan(0);
        RuleFor(x => x.MinimumNumOfCores)
            .GreaterThan(0);
    }
}