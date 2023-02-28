using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Responses;

namespace Middleware.RedisInterface.Mappings;

public static class DomainToApiContractMapper
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

    public static ContainerResponse ToContainerResponse(this ContainerImageModel x)
    {
        return new ContainerResponse()
        {
            Id = x.Id,
            Name = x.Name,
            Description = x.Description,
            K8sDeployment = x.K8SDeployment,
            K8sService = x.K8SService,
            LastUpdateTime = x.Timestamp
        };
    }

    public static GetAllContainersResponse ToContainersResponse(this IEnumerable<ContainerImageModel> contaiers)
    {
        return new GetAllContainersResponse()
        {
            Containers = contaiers.Select(x => x.ToContainerResponse())
        };
    }

    public static EdgeResponse ToEdgeResponse(this EdgeModel x)
    {
        return new EdgeResponse()
        {
            Id = x.Id,
            Name = x.Name,
            Status = x.EdgeStatus,
            IpAddress = x.EdgeIp,
            MacAddress = x.MacAddress,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            LastUpdatedTime = x.LastUpdatedTime
        };
    }

    public static GetAllEdgesResponse ToEdgesResponse(this IEnumerable<EdgeModel> edges)
    {
        return new GetAllEdgesResponse()
        {
            Edges = edges.Select(x => x.ToEdgeResponse())
        };
    }
}