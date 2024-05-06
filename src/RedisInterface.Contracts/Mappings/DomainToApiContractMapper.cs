using Middleware.Models.Domain;
using Middleware.Models.Domain.Ros;
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
            SingleNetAppEntryPoint = x.SingleNetAppEntryPoint,
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
            LastUpdatedTime = x.LastUpdatedTime,
            Latency = x.Latency,
            Throughput = x.Throughput
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
            LastUpdatedTime = x.LastUpdatedTime,
            Latency = x.Latency,
            Throughput = x.Throughput
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
            IsPersistent = x.IsPersistent,
            MinimumRam = x?.Ram?.Minimal,
            MinimumNumOfCores = (int?)x.NumberOfCores!.Minimal,
            Ram = new(x.Ram!.Minimal, x.Ram.Optimal, x.Ram.Priority.ToString()),
            NumberOfCores = new(x.NumberOfCores.Minimal, x.NumberOfCores.Optimal, x.NumberOfCores.Priority.ToString()),
            DiskStorage = new(x.DiskStorage!.Minimal, x.DiskStorage.Optimal, x.DiskStorage.Priority.ToString()),
            Throughput = new(x.Throughput!.Minimal, x.Throughput.Optimal, x.Throughput.Priority.ToString()),
            Latency = new(x.Latency!.Minimal, x.Latency.Optimal, x.Latency.Priority.ToString()),
            RosVersion = x.RosVersion,
            RosDistro = x.RosDistro,
            RosTopicPublishers = x.RosTopicsPub.Select(t => t.ToRosTopicResponse()),
            RosTopicSubscribers = x.RosTopicsSub.Select(t => t.ToRosTopicResponse()),
            RosActions = x.Actions.Select(t => t.ToRosActionResponse()).ToList(),
            RosServices = x.Services.Select(t=>t.ToRosServiceResponse()).ToList(),
            RosTransforms = x.Transforms.Select(t=>t.ToRosTransformsResponse()).ToList(),
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
        return new()
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

    public static RosTopicResponse ToRosTopicResponse(this RosTopicModel x)
    {
        return new()
        {
            Name = x.Name,
            Type = x.Type,
            Description = x.Description,
            Enabled = x.Enabled,
            Compression = x.Compression,
            Qos = x.Qos?.ToQosResponse()
        };
    }

    public static RosQosResponse ToQosResponse(this Qos x)
    {
        return new()
        {
            Deadline = x.Deadline,
            Depth = x.Depth,
            Durability = x.Durability,
            History = x.History,
            Lease = x.Lease,
            Lifespan = x.Lifespan,
            Liveliness = x.Liveliness,
            Preset = x.Preset,
            Reliability = x.Reliability
        };
    }

    public static RosQosRequest ToQosRequest(this Qos x)
    {
        return new()
        {
            Deadline = x.Deadline,
            Depth = x.Depth,
            Durability = x.Durability,
            History = x.History,
            Lease = x.Lease,
            Lifespan = x.Lifespan,
            Liveliness = x.Liveliness,
            Preset = x.Preset,
            Reliability = x.Reliability
        };
    }

    public static RosTopicRequest ToRosTopicRequest(this RosTopicModel x)
    {
        return new()
        {
            Name = x.Name,
            Type = x.Type,
            Description = x.Description,
            Enabled = x.Enabled,
            Compression = x.Compression,
            Qos = x.Qos?.ToQosRequest()
        };
    }

    public static FullActionResponse ToFullActionResponse(this ActionModel x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Priority = x.ActionPriority,
            Order = x.Order,
            MinimumRam = x.MinimumRam,
            MinimumNumCores = x.MinimumNumCores,
            SingleNetAppEntryPoint = x.SingleNetAppEntryPoint,
            Tags = x.Tags,
            Services = x.Services?.Select(s => s.ToFullInstanceResponse())
        };
    }

    public static FullInstanceResponse ToFullInstanceResponse(this InstanceModel x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Family = x.InstanceFamily,
            Type = x.ServiceType,
            IsReusable = x.IsReusable,
            IsPersistent = x.IsPersistent,
            MinimumRam = x?.Ram?.Minimal,
            MinimumNumOfCores = (int)x.NumberOfCores!.Minimal,
            Ram = new(x.Ram!.Minimal, x.Ram.Optimal, x.Ram.Priority.ToString()),
            NumberOfCores = new(x.NumberOfCores.Minimal, x.NumberOfCores.Optimal, x.NumberOfCores.Priority.ToString()),
            DiskStorage = new(x.DiskStorage!.Minimal, x.DiskStorage.Optimal, x.DiskStorage.Priority.ToString()),
            Throughput = new(x.Throughput!.Minimal, x.Throughput.Optimal, x.Throughput.Priority.ToString()),
            Latency = new(x.Latency!.Minimal, x.Latency.Optimal, x.Latency.Priority.ToString()),
            RosVersion = x.RosVersion,
            RosDistro = x.RosDistro,
            RosTopicPublishers = x.RosTopicsPub.Select(t => t.ToRosTopicResponse()),
            RosTopicSubscribers = x.RosTopicsSub.Select(t => t.ToRosTopicResponse()),
            RosActions = x.Actions.Select(t => t.ToRosActionResponse()).ToList(),
            RosServices = x.Services.Select(t=>t.ToRosServiceResponse()).ToList(),
            RosTransforms = x.Transforms.Select(t=>t.ToRosTransformsResponse()).ToList(),
            Tags = x.Tags,
            OnboardedTime = x.OnboardedTime,
            AppliedPolicies = x.AppliedPolicies,
            ContainerImage = x.ContainerImage?.ToFullContainerResponse()
        };
    }

    public static FullContainerResponse ToFullContainerResponse(this ContainerImageModel x)
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

    public static FullTaskResponse ToFullTaskResponse(this TaskModel x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Priority = ((Priority)x.TaskPriority).ToString(),
            IsDeterministic = x.DeterministicTask,
            Tags = x.Tags,
            ActionSequence = x.ActionSequence?.Select(s => s.ToFullActionResponse())
        };
    }

    public static LocationResponse ToLocationResponse(this Location x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Organization = x.Organization,
            Type = x.Type.ToString(),
            IpAddress = x.Address,
            MacAddress = x.MacAddress,
            Status = x.Status,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            IsOnline = x.IsOnline,
            LastUpdatedTime = x.LastUpdatedTime,
            Latency = x.Latency,
            Throughput = x.Throughput
        };
    }

    public static GetLocationsResponse ToLocationsResponse(this IEnumerable<Location> x)
    {
        return new()
        {
            Locations = x.Select(l => l.ToLocationResponse())
        };
    }

    public static CloudResponse ToCloudResponse(this Location x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Organization = x.Organization,
            Type = x.Type.ToString(),
            IpAddress = x.Address,
            MacAddress = x.MacAddress,
            Status = x.Status,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            LastUpdatedTime = x.LastUpdatedTime,
            Latency = x.Latency,
            Throughput = x.Throughput
        };
    }

    public static GetCloudsResponse ToCloudsResponse(this IEnumerable<Location> x)
    {
        return new()
        {
            Clouds = x.Select(l => l.ToCloudResponse())
        };
    }

    public static EdgeResponse ToEdgeResponse(this Location x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Organization = x.Organization,
            Type = x.Type.ToString(),
            IpAddress = x.Address,
            MacAddress = x.MacAddress,
            Status = x.Status,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            LastUpdatedTime = x.LastUpdatedTime,
            Latency = x.Latency,
            Throughput = x.Throughput
        };
    }

    public static GetEdgesResponse ToEdgesResponse(this IEnumerable<Location> x)
    {
        return new()
        {
            Edges = x.Select(l => l.ToEdgeResponse())
        };
    }

    public static RosActionRequest ToRosActionRequest(this RosActionModel x)
    {
        return new()
        {
            Name = x.Name,
            Type = x.Type
        };
    }

    public static RosActionResponse ToRosActionResponse(this RosActionModel x)
    {
        return new()
        {
            Name = x.Name,
            Type = x.Type
        };
    }
    public static RosServiceResponse ToRosServiceResponse(this RosServiceModel x)
    {
        return new()
        {
            Name = x.Name,
            Type = x.Type,
            Description = x.Description,
            Qos = x.Qos?.ToQosResponse()
        };
    } 
    public static RosServiceRequest ToRosServiceRequest(this RosServiceModel x)
    {
        return new()
        {
            Name = x.Name,
            Type = x.Type,
            Description = x.Description,
            Qos = x.Qos?.ToQosRequest()
        };
    } 
    public static RosTransformsResponse ToRosTransformsResponse(this RosTransformsModel x)
    {
        return new()
        {
            AngularThres = x.AngularThres,
            SourceFrame = x.SourceFrame,
            TargetFrame = x.TargetFrame,
            TransThres = x.TransThres,
            MaxPublishPeriod = x.MaxPublishPeriod
        };
    }
    public static RosTransformsRequest ToRosTransformsRequest(this RosTransformsModel x)
    {
        return new()
        {
            AngularThres = x.AngularThres,
            SourceFrame = x.SourceFrame,
            TargetFrame = x.TargetFrame,
            TransThres = x.TransThres,
            MaxPublishPeriod = x.MaxPublishPeriod
        };
    }
    public static SystemConfigResponse ToSystemConfigResponse(this SystemConfigModel x)
    {
        return new()
        {
            HeartbeatExpirationInMinutes = x.HeartbeatExpirationInMinutes,
            Ros1RelayContainer = x.Ros1RelayContainer,
            Ros2RelayContainer = x.Ros2RelayContainer,
            RosInterRelayNetAppContainer = x.RosInterRelayNetAppContainer,
            HermesContainer = x.HermesContainer,
            S3DataPersistenceRegion = x.S3DataPersistenceRegion,
            S3DataPersistenceBucketName = x.S3DataPersistenceBucketName
        };
    }
}