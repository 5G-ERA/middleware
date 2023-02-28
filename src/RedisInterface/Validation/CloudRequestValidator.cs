using System.Data;
using FluentValidation;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Validation;

public class CloudRequestValidator : AbstractValidator<CloudRequest>
{
    public CloudRequestValidator()
    {
        RuleFor(x => x.Name)
            .NotNull()
            .NotEmpty();
        RuleFor(x => x.IpAddress)
            .NotNull();
        RuleFor(x => x.Status)
            .NotNull();
        
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