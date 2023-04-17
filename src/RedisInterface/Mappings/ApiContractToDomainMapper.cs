using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Requests;

namespace Middleware.RedisInterface.Mappings;

public static class ApiContractToDomainMapper
{
    public static ActionModel ToAction(this UpdateActionRequest x)
    {
        return new ActionModel()
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
        return new CloudModel()
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
            LastUpdatedTime = DateTime.Now
        };
    }

    public static ContainerImageModel ToContainer(this UpdateContainerRequest x)
    {
        return new ContainerImageModel()
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
        return new EdgeModel()
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
            LastUpdatedTime = DateTime.Now
        };
    }

    public static InstanceModel ToInstance(this UpdateInstanceRequest x)
    {
        return new InstanceModel()
        {
            Id = x.Id,
            Name = x.Instance.Name,
            InstanceFamily = x.Instance.Family,
            Tags = x.Instance.Tags?.ToList(),
            IsReusable = x.Instance.IsReusable,
            ServiceType = x.Instance.Type,
            MinimumRam = x.Instance.MinimumRam,
            MinimumNumCores = x.Instance.MinimumNumOfCores,
            OnboardedTime = DateTime.Now,
            RosVersion = x.Instance.RosVersion,
            ROSDistro = x.Instance.RosDistro,
            RosTopicsPub = x.Instance.RosTopicPublishers.ToList(),
            RosTopicsSub = x.Instance.RosTopicSubscribers.ToList()
        };
    }
    public static PolicyModel ToPolicy(this UpdatePolicyRequest x)
    {
        return new PolicyModel()
        {
            Id = x.Id,
            Name = x.Policy.Name,
            Description = x.Policy.Description,
            Type = x.Policy.Type,
            IsActive = x.Policy.IsActive,
            IsExclusiveWithinType = x.Policy.IsExclusiveWithinType,
            Timestamp = x.Policy.LastTimeUpdated
        };
    }
    public static RobotModel ToRobot(this UpdateRobotRequest x)
    {
        return new RobotModel()
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
            OnboardedTime = DateTime.Now,
        };
    }
    public static TaskModel ToTask(this UpdateTaskRequest x)
    {
        return new TaskModel()
        {
            Id = x.Id,
            Name = x.Task.Name,
            TaskPriority = (int)Enum.Parse<Priority>(x.Task.Priority),
            DeterministicTask = x.Task.IsDeterministic,
            Tags = x.Task.Tags.ToList()
        };
    }
}