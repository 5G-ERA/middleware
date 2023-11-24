using FluentValidation;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Validation;

public class LocationRequestValidator : AbstractValidator<LocationRequest>
{
    public LocationRequestValidator()
    {
        RuleFor(x => x.Name)
            .NotNull()
            .NotEmpty();
        RuleFor(x => x.Organization)
            .NotNull()
            .NotEmpty();
        RuleFor(x => x.IpAddress)
            .NotNull();
        RuleFor(x => x.Status)
            .NotNull();
        RuleFor(x => x.Type)
            .NotNull().IsEnumName(typeof(LocationType), false)
            .WithMessage(x => $"{x.Type} is not a valid Locomotion type name. " +
                              $"Valid options are: {string.Join(", ", Enum.GetNames<LocationType>())}");
        RuleFor(x => x.Cpu)
            .GreaterThan(0);
        RuleFor(x => x.Ram)
            .GreaterThan(0);
        RuleFor(x => x.DiskStorage)
            .GreaterThan(0);
        RuleFor(x => x.VirtualRam)
            .GreaterThan(0);
        RuleFor(x => x.NumberOfCores)
            .GreaterThan(0);
    }
}