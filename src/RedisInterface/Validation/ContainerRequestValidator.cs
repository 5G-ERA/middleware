using FluentValidation;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Validation;

public class ContainerRequestValidator : AbstractValidator<ContainerRequest>
{
    public ContainerRequestValidator()
    {
        RuleFor(x => x.Name)
            .NotNull().NotEmpty();
        
        // TODO: add custom validator if is valid kubernetes object
        RuleFor(x => x.K8SDeployment)
            .NotNull().NotEmpty();
        
    }
}