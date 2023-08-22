using FluentValidation;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Requests;

namespace Middleware.RedisInterface.Validation;

public class UpdatePolicyRequestValidator : AbstractValidator<UpdatePolicyRequest>
{
    public UpdatePolicyRequestValidator(PolicyRequestValidator prv)
    {
        RuleFor(x => x.Policy).SetValidator(prv);        
    }
}