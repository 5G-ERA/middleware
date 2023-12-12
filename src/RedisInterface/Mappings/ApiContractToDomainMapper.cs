using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Requests;

namespace Middleware.RedisInterface.Mappings;

public static class ApiContractToDomainMapper
{
    public static ActionModel ToAction(this UpdateActionRequest x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Action.Name,
            Order = x.Action.Order,
            MinimumRam = x.Action.MinimumRam,
            MinimumNumCores = x.Action.MinimumNumCores,
            Tags = x.Action.Tags.ToList()
        };
    }

    public static CloudModel ToCloud(this UpdateCloudRequest x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Cloud.Name,
            CloudIp = x.Cloud.IpAddress,
            MacAddress = x.Cloud.MacAddress,
            CloudStatus = x.Cloud.Status,
            Cpu = x.Cloud.Cpu,
            NumberOfCores = x.Cloud.NumberOfCores,
            Ram = x.Cloud.Ram,
            VirtualRam = x.Cloud.VirtualRam,
            DiskStorage = x.Cloud.DiskStorage,
            LastUpdatedTime = DateTime.Now,
            Organization = x.Cloud.Organization,
            Latency = x.Cloud.Latency,
            Throughput = x.Cloud.Throughput
        };
    }

    public static ContainerImageModel ToContainer(this UpdateContainerRequest x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Container.Name,
            Description = x.Container.Description,
            K8SDeployment = x.Container.K8SDeployment,
            K8SService = x.Container.K8SService,
            Timestamp = DateTime.Now
        };
    }

    public static EdgeModel ToEdge(this UpdateEdgeRequest x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Edge.Name,
            EdgeIp = x.Edge.IpAddress,
            MacAddress = x.Edge.MacAddress,
            EdgeStatus = x.Edge.Status,
            Cpu = x.Edge.Cpu,
            NumberOfCores = x.Edge.NumberOfCores,
            Ram = x.Edge.Ram,
            VirtualRam = x.Edge.VirtualRam,
            DiskStorage = x.Edge.DiskStorage,
            LastUpdatedTime = DateTime.Now,
            Organization = x.Edge.Organization,
            Latency = x.Edge.Latency,
            Throughput = x.Edge.Throughput
        };
    }

    public static InstanceModel ToInstance(this UpdateInstanceRequest x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Instance.Name,
            InstanceFamily = x.Instance.Family,
            Tags = x.Instance.Tags?.ToList(),
            IsReusable = x.Instance.IsReusable,
            ServiceType = x.Instance.Type,
            Ram = new(x.Instance.Ram.Minimum, x.Instance.Ram.Optimal, x.Instance.Ram.Priority, true),
            NumberOfCores = new(x.Instance.NumberOfCores.Minimum, x.Instance.NumberOfCores.Optimal,
                x.Instance.NumberOfCores.Priority, true),
            DiskStorage = new(x.Instance.DiskStorage.Minimum, x.Instance.DiskStorage.Optimal,
                x.Instance.DiskStorage.Priority, true),
            Throughput = new(x.Instance.Throughput.Minimum, x.Instance.Throughput.Optimal,
                x.Instance.Throughput.Priority, true),
            Latency = new(x.Instance.Latency.Minimum, x.Instance.Latency.Optimal, x.Instance.Latency.Priority, false),
            OnboardedTime = DateTime.Now,
            RosVersion = x.Instance.RosVersion,
            RosDistro = x.Instance.RosDistro,
            RosTopicsPub = x.Instance.RosTopicPublishers.Select(t => t.ToRosTopic()).ToList(),
            RosTopicsSub = x.Instance.RosTopicSubscribers.Select(t => t.ToRosTopic()).ToList()
        };
    }

    public static PolicyModel ToPolicy(this UpdatePolicyRequest x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Policy.Name,
            Description = x.Policy.Description,
            Type = Enum.Parse<PolicyType>(x.Policy.Type),
            Scope = Enum.Parse<PolicyScope>(x.Policy.Scope),
            IsActive = x.Policy.IsActive,
            IsExclusiveWithinType = x.Policy.IsExclusiveWithinType,
            Timestamp = x.Policy.LastTimeUpdated
        };
    }

    public static PolicyModel ToLimitedPolicy(this UpdatePolicyRequest x)
    {
        return new()
        {
            Id = x.Id,
            IsActive = x.Policy.IsActive,
            Priority = Enum.Parse<Priority>(x.Policy.Priority)
        };
    }

    public static RobotModel ToRobot(this UpdateRobotRequest x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Robot.Name,
            RobotModelName = x.Robot.ModelName,
            RobotStatus = x.Robot.Status,
            BatteryStatus = x.Robot.BatteryStatus,
            RosVersion = x.Robot.RosVersion,
            RosDistro = x.Robot.RosDistro,
            ROSRepo = x.Robot.RosRepo,
            ROSNodes = x.Robot.RosNodes?.ToList(),
            MaximumPayload = x.Robot.MaximumPayload,
            MaximumRotationalVelocity = x.Robot.MaximumRotationalVelocity,
            MaximumTranslationalVelocity = x.Robot.MaximumTranslationalVelocity,
            RobotWeight = x.Robot.RobotWeight,
            Manufacturer = x.Robot.Manufacturer,
            ManufacturerUrl = x.Robot.ManufacturerUrl,
            MacAddress = x.Robot.MacAddress,
            LocomotionSystem = x.Robot.LocomotionSystem,
            LocomotionType = x.Robot.LocomotionType,
            Sensors = x.Robot.Sensors?.ToList(),
            Actuators = x.Robot.Actuators?.ToList(),
            Manipulators = x.Robot.Manipulators?.ToList(),
            Cpu = x.Robot.Cpu,
            NumberCores = x.Robot.NumberOfCores,
            Ram = x.Robot.Ram,
            StorageDisk = x.Robot.StorageDisk,
            Questions = x.Robot.Questions?.ToList(),
            LastUpdatedTime = DateTime.Now,
            OnboardedTime = DateTime.Now
        };
    }

    public static TaskModel ToTask(this UpdateTaskRequest x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Task.Name,
            TaskPriority = (int)Enum.Parse<Priority>(x.Task.Priority),
            DeterministicTask = x.Task.IsDeterministic,
            Tags = x.Task.Tags.ToList()
        };
    }

    public static Location ToLocation(this UpdateLocationRequest x)
    {
        var loc = x.Location;
        return new()
        {
            Id = x.Id,
            Address = loc.IpAddress,
            Type = Enum.Parse<LocationType>(loc.Type),
            Name = loc.Name,
            Cpu = loc.Cpu,
            DiskStorage = loc.DiskStorage,
            MacAddress = loc.MacAddress,
            NumberOfCores = loc.NumberOfCores,
            Organization = loc.Organization,
            Ram = loc.Ram,
            Status = loc.Status,
            VirtualRam = loc.VirtualRam,
            Latency = loc.Latency,
            Throughput = loc.Throughput
        };
    }

    public static Location ToLocation(this UpdateCloudRequest x)
    {
        var loc = x.Cloud;
        return new()
        {
            Id = x.Id,
            Address = loc.IpAddress,
            Type = LocationType.Cloud,
            Name = loc.Name,
            Cpu = loc.Cpu,
            DiskStorage = loc.DiskStorage,
            MacAddress = loc.MacAddress,
            NumberOfCores = loc.NumberOfCores,
            Organization = loc.Organization,
            Ram = loc.Ram,
            Status = loc.Status,
            VirtualRam = loc.VirtualRam,
            Latency = loc.Latency,
            Throughput = loc.Throughput
        };
    }

    public static Location ToLocation(this UpdateEdgeRequest x)
    {
        var loc = x.Edge;
        return new()
        {
            Id = x.Id,
            Address = loc.IpAddress,
            Type = LocationType.Edge,
            Name = loc.Name,
            Cpu = loc.Cpu,
            DiskStorage = loc.DiskStorage,
            MacAddress = loc.MacAddress,
            NumberOfCores = loc.NumberOfCores,
            Organization = loc.Organization,
            Ram = loc.Ram,
            Status = loc.Status,
            VirtualRam = loc.VirtualRam,
            Latency = loc.Latency,
            Throughput = loc.Throughput
        };
    }
}