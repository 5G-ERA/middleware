using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Responses;

namespace Middleware.RedisInterface.Mappings;

public static  class DomainToApiContractMapper
{
    public static ActionResponse ToActionResponse(this ActionModel x)
    {
        return new ActionResponse()
        {
            Id = x.Id,
            Name = x.Name,
            Priority = x.ActionPriority,
            Order = x.Order,
            MinimumRam = x.MinimumRam,
            MinimumNumCores = x.MinimumNumCores,
            Tags = x.Tags
        };
    }
    public static GetAllActionsResponse ToActionsResponse(this IEnumerable<ActionModel> actions)
    {
        return new GetAllActionsResponse()
        {
            Actions = actions.Select(x => new ActionResponse
            {
                Id = x.Id,
                Name = x.Name,
                Order = x.Order,
                MinimumRam = x.MinimumRam,
                MinimumNumCores = x.MinimumNumCores,
                Tags = x.Tags
            })
        };
    }

}