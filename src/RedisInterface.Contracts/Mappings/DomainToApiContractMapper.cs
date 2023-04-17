using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Responses;

namespace Middleware.RedisInterface.Contracts.Mappings;

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

    public static GetActionsResponse ToActionsResponse(this IEnumerable<ActionModel> actions)
    {
        return new GetActionsResponse()
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
            Type = LocationType.Cloud.ToString(),
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

    public static GetCloudsResponse ToCloudsResponse(this IEnumerable<CloudModel> clouds)
    {
        return new GetCloudsResponse()
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

    public static GetContainersResponse ToContainersResponse(this IEnumerable<ContainerImageModel> containers)
    {
        return new GetContainersResponse()
        {
            Containers = containers.Select(x => x.ToContainerResponse())
        };
    }

    public static EdgeResponse ToEdgeResponse(this EdgeModel x)
    {
        return new EdgeResponse()
        {
            Id = x.Id,
            Name = x.Name,
            Type = LocationType.Edge.ToString(),
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

    public static GetEdgesResponse ToEdgesResponse(this IEnumerable<EdgeModel> edges)
    {
        return new GetEdgesResponse()
        {
            Edges = edges.Select(x => x.ToEdgeResponse())
        };
    }

    public static InstanceResponse ToInstanceResponse(this InstanceModel x)
    {
        return new InstanceResponse()
        {
            Id = x.Id,
            Name = x.Name,
            Family = x.InstanceFamily,
            Type = x.ServiceType,
            IsReusable = x.IsReusable,
            MinimumRam = x.MinimumRam,
            MinimumNumOfCores = x.MinimumNumCores,
            RosVersion = x.RosVersion,
            RosDistro = x.ROSDistro,
            RosTopicPublishers = x.RosTopicsPub,
            RosTopicSubscribers = x.RosTopicsSub,
            Tags = x.Tags,
            OnboardedTime = x.OnboardedTime
        };
    }
    public static GetInstancesResponse ToInstancesResponse(this IEnumerable<InstanceModel> instances)
    {
        return new GetInstancesResponse()
        {
            Instances = instances.Select(x => x.ToInstanceResponse())
        };
    }

    public static PolicyResponse ToPolicyResponse(this PolicyModel x)
    {
        return new PolicyResponse()
        {
            Id = x.Id,
            Name = x.Name,
            Description = x.Description,
            Type = x.Type,
            IsActive = x.IsActive,
            IsExclusiveWithinType = x.IsExclusiveWithinType,
            LastTimeUpdated = x.Timestamp
        };
    }

    public static GetPoliciesResponse ToPoliciesResponse(this IEnumerable<PolicyModel> policies)
    {
        return new GetPoliciesResponse()
        {
            Policies = policies.Select(x => x.ToPolicyResponse())
        };
    }

    public static RobotResponse ToRobotResponse(this RobotModel x)
    {
        return new RobotResponse()
        {
            Id = x.Id,
            Name = x.Name,
            ModelName = x.RobotModelName,
            Status = x.RobotStatus,
            BatteryStatus = x.BatteryStatus,
            RosVersion = x.RosVersion,
            RosDistro = x.RosDistro,
            RosRepo = x.ROSRepo,
            RosNodes = x.ROSNodes,
            MaximumPayload = x.MaximumPayload,
            MaximumTranslationalVelocity = x.MaximumTranslationalVelocity,
            MaximumRotationalVelocity = x.MaximumRotationalVelocity,
            RobotWeight = x.RobotWeight,
            Manufacturer = x.Manufacturer,
            ManufacturerUrl = x.ManufacturerUrl,
            MacAddress = x.MacAddress,
            LocomotionSystem = x.LocomotionSystem,
            LocomotionType = x.LocomotionType,
            Sensors = x.Sensors,
            Actuators = x.Actuators,
            Manipulators = x.Manipulators,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberCores,
            Ram = x.Ram,
            StorageDisk = x.StorageDisk,
            Questions = x.Questions,
            LastUpdatedTime = x.LastUpdatedTime,
            OnboardedTime = x.OnboardedTime
        };
    }
    public static GetRobotsResponse ToRobotsResponse(this IEnumerable<RobotModel> robots)
    {
        return new GetRobotsResponse()
        {
            Robots = robots.Select(x => x.ToRobotResponse())
        };
    }

    public static TaskResponse ToTaskResponse(this TaskModel x)
    {
        return new TaskResponse()
        {
            Id = x.Id,
            Name = x.Name,
            IsDeterministic = x.DeterministicTask,
            Priority = ((Priority)x.TaskPriority).ToString(),
            Tags = x.Tags
        };
    }

    public static GetTasksResponse ToTasksResponse(this IEnumerable<TaskModel> tasks)
    {
        return new GetTasksResponse()
        {
            Tasks = tasks.Select(x => x.ToTaskResponse())
        };
    }
}