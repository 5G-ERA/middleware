using FluentValidation;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Validation;

public class InstanceRequestValidator : AbstractValidator<InstanceRequest>
{
    public InstanceRequestValidator()
    {
        var distroNames = RosDistroHelper.GetRosDistroNames();
        RuleFor(x => x.Name)
            .NotNull().NotEmpty();
        RuleFor(x => x.Type)
            .NotNull().NotEmpty();
        RuleFor(x => x.IsReusable)
            .NotNull();
        RuleForEach(x => x.RosTopicPublishers)
            .NotNull().NotEmpty().SetValidator(new RosTopicValidator());
        RuleForEach(x => x.RosTopicSubscribers)
            .NotNull().NotEmpty().SetValidator(new RosTopicValidator());
        RuleFor(x => x.RosVersion)
            .NotNull().InclusiveBetween(1, 2);
        RuleFor(x => x.RosDistro)
            .NotNull().NotEmpty()
            //.IsEnumName(typeof(RosDistro), caseSensitive: false)
            .Custom((value, context) =>
            {
                if (!distroNames.Contains(value))
                {
                    context.AddFailure($"'{value}' is not a valid ROS distribution name" +
                                       $"Valid options are: {string.Join(", ", distroNames)}");
                }
            });
        RuleFor(x => x.Family)
            .NotNull().NotEmpty();
        RuleFor(x => x.MinimumRam)
            .GreaterThan(0);
        RuleFor(x => x.MinimumNumOfCores)
            .GreaterThan(0);
    }
}