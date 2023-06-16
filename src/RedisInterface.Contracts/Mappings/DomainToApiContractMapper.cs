using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;

namespace Middleware.RedisInterface.Contracts.Mappings;

public static class DomainToApiContractMapper
{
    public static ActionResponse ToActionResponse(this ActionModel x)
    {
        return new()
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
        return new()
        {
            Actions = actions.Select(x => x.ToActionResponse())
        };
    }

    public static CloudResponse ToCloudResponse(this CloudModel x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Organization = x.Organization,
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
        return new()
        {
            Clouds = clouds.Select(x => x.ToCloudResponse())
        };
    }

    public static ContainerResponse ToContainerResponse(this ContainerImageModel x)
    {
        return new()
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
        return new()
        {
            Containers = containers.Select(x => x.ToContainerResponse())
        };
    }

    public static EdgeResponse ToEdgeResponse(this EdgeModel x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Organization = x.Organization,
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
        return new()
        {
            Edges = edges.Select(x => x.ToEdgeResponse())
        };
    }

    public static InstanceResponse ToInstanceResponse(this InstanceModel x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Family = x.InstanceFamily,
            Type = x.ServiceType,
            IsReusable = x.IsReusable,
            MinimumRam = x.MinimumRam,
            MinimumNumOfCores = x.MinimumNumCores,
            RosVersion = x.RosVersion,
            RosDistro = x.RosDistro,
            RosTopicPublishers = x.RosTopicsPub,
            RosTopicSubscribers = x.RosTopicsSub,
            Tags = x.Tags,
            OnboardedTime = x.OnboardedTime,
            AppliedPolicies = x.AppliedPolicies
        };
    }

    public static GetInstancesResponse ToInstancesResponse(this IEnumerable<InstanceModel> instances)
    {
        return new()
        {
            Instances = instances.Select(x => x.ToInstanceResponse())
        };
    }

    public static PolicyResponse ToPolicyResponse(this PolicyModel x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Description = x.Description,
            Type = x.Type.ToString(),
            Scope = x.Scope.ToString(),
            IsActive = x.IsActive,
            IsExclusiveWithinType = x.IsExclusiveWithinType,
            LastTimeUpdated = x.Timestamp,
            Priority = x.Priority.ToString()
        };
    }

    public static GetPoliciesResponse ToPoliciesResponse(this IEnumerable<PolicyModel> policies)
    {
        return new()
        {
            Policies = policies.Select(x => x.ToPolicyResponse())
        };
    }

    public static RobotResponse ToRobotResponse(this RobotModel x)
    {
        return new()
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
            OnboardedTime = x.OnboardedTime,
            SimCardNumber = x.SimCardNumber
        };
    }

    public static GetRobotsResponse ToRobotsResponse(this IEnumerable<RobotModel> robots)
    {
        return new()
        {
            Robots = robots.Select(x => x.ToRobotResponse())
        };
    }

    public static TaskResponse ToTaskResponse(this TaskModel x)
    {
        return new()
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
        return new()
        {
            Tasks = tasks.Select(x => x.ToTaskResponse())
        };
    }

    public static SliceResponse ToSliceResponse(this SliceModel x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Site = x.Site,
            SliceType = x.SliceType.ToString(),
            TrafficType = x.TrafficType.ToString(),
            ExpDataRateDl = x.ExpDataRateDl,
            ExpDataRateUl = x.ExpDataRateDl,
            Jitter = x.Jitter,
            Latency = x.Latency,
            UserDensity = x.UserDensity,
            UserSpeed = x.UserSpeed,
            Imsi = x.Imsi
        };
    }

    public static GetSlicesResponse ToSlicesResponse(this IEnumerable<SliceModel> slices)
    {
        return new()
        {
            Slices = slices.Select(x => x.ToSliceResponse())
        };
    }
    public static SliceRequest ToSliceRequest(this SliceModel x)
    {
        return new SliceRequest
        {
            SliceId = x.Name,
            Site = x.Site,
            TrafficType = x.TrafficType.ToString(),
            ExpDataRateDl = x.ExpDataRateDl,
            ExpDataRateUl = x.ExpDataRateDl,
            Jitter = x.Jitter,
            Latency = x.Latency,
            UserDensity = x.UserDensity,
            UserSpeed = x.UserSpeed,
            Imsi = x.Imsi
        };
    }
}