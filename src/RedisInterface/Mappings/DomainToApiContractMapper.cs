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
            Actions = actions.Select(x => x.ToActionResponse())
        };
    }

    public static CloudResponse ToCloudResponse(this CloudModel x)
    {
        return new CloudResponse()
        {
            Id = x.Id,
            Name = x.Name,
            Type = x.Type,
            IpAddress = x.CloudIp,
            MacAddress = x.MacAddress,
            Status = x.CloudStatus,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            LastUpdatedTime = x.LastUpdatedTime
        };
    } 
    public static GetAllCloudsResponse ToCloudsResponse(this IEnumerable<CloudModel> clouds)
    {
        return new GetAllCloudsResponse()
        {
            Clouds = clouds.Select(x => x.ToCloudResponse())
        };
    } 
}