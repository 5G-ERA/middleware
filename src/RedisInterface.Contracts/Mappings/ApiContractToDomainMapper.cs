using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Contracts.Mappings;

public static class ApiContractToDomainMapper
{
    public static ActionModel ToAction(this ActionRequest x)
    {
        return new ActionModel()
        {
            Name = x.Name,
            Order = x.Order,
            MinimumRam = x.MinimumRam,
            MinimumNumCores = x.MinimumNumCores,
            Tags = x.Tags.ToList()
        };
    }

    public static CloudModel ToCloud(this CloudRequest x)
    {
        return new CloudModel()
        {
            Name = x.Name,
            Type = x.Type,
            CloudIp = x.IpAddress,
            MacAddress = x.MacAddress,
            CloudStatus = x.Status,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            LastUpdatedTime = DateTime.Now
        };
    }

    public static ContainerImageModel ToContainer(this ContainerRequest x)
    {
        return new ContainerImageModel()
        {
            Name = x.Name,
            Description = x.Description,
            K8SDeployment = x.K8SDeployment,
            K8SService = x.K8SService,
            Timestamp = DateTime.Now
        };
    }
    
    public static EdgeModel ToEdge(this EdgeRequest x)
    {
        return new EdgeModel()
        {
            Name = x.Name,
            EdgeIp = x.IpAddress,
            MacAddress = x.MacAddress,
            EdgeStatus = x.Status,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            LastUpdatedTime = DateTime.Now
        };
    }
    
    public static InstanceModel ToInstance(this InstanceRequest x)
    {
        return new InstanceModel()
        {
            Name = x.Name,
            InstanceFamily = x.Family,
            Tags = x.Tags?.ToList(),
            IsReusable = x.IsReusable,
            ServiceType = x.Type,
            MinimumRam = x.MinimumRam,
            MinimumNumCores = x.MinimumNumOfCores,
            OnboardedTime = DateTime.Now,
            RosVersion = x.RosVersion,
            ROSDistro = x.RosDistro,
            RosTopicsPub = x.RosTopicPublishers.ToList(),
            RosTopicsSub = x.RosTopicSubscribers.ToList()
        };
    }

    public static PolicyModel ToPolicy(this PolicyRequest x)
    {
        return new PolicyModel()
        {
            Name = x.Name,
            Description = x.Description,
            Type = x.Type,
            IsActive = x.IsActive,
            IsExclusiveWithinType = x.IsExclusiveWithinType,
            Timestamp = x.LastTimeUpdated
        };
    }

    public static RobotModel ToRobot(this RobotRequest x)
    {
        return new RobotModel()
        {
            Name = x.Name,
            RobotModelName = x.ModelName,
            RobotStatus = x.Status,
            BatteryStatus = x.BatteryStatus,
            RosVersion = x.RosVersion,
            RosDistro = x.RosDistro,
            ROSRepo = x.RosRepo,
            ROSNodes = x.RosNodes?.ToList(),
            MaximumPayload = x.MaximumPayload,
            MaximumRotationalVelocity = x.MaximumRotationalVelocity,
            MaximumTranslationalVelocity = x.MaximumTranslationalVelocity,
            RobotWeight = x.RobotWeight,
            Manufacturer = x.Manufacturer,
            ManufacturerUrl = x.ManufacturerUrl,
            MacAddress = x.MacAddress,
            LocomotionSystem = x.LocomotionSystem,
            LocomotionType = x.LocomotionType,
            Sensors = x.Sensors?.ToList(),
            Actuators = x.Actuators?.ToList(),
            Manipulators = x.Manipulators?.ToList(),
            Cpu = x.Cpu,
            NumberCores = x.NumberOfCores,
            Ram = x.Ram,
            StorageDisk = x.StorageDisk,
            Questions = x.Questions?.ToList(),
            LastUpdatedTime = DateTime.Now
        };
    }

    public static TaskModel ToTask(this TaskRequest x)
    {
        return new TaskModel()
        {
            Name = x.Name,
            TaskPriority = (int)Enum.Parse<Priority>(x.Priority, true),
            DeterministicTask = x.IsDeterministic,
            Tags = x.Tags.ToList()
        };
    }
}