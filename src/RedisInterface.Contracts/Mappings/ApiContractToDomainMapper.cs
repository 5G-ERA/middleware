﻿using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;

namespace Middleware.RedisInterface.Contracts.Mappings;

public static class ApiContractToDomainMapper
{
    public static ActionModel ToAction(this ActionRequest x)
    {
        return new()
        {
            Name = x.Name,
            Order = x.Order,
            MinimumRam = x.MinimumRam,
            MinimumNumCores = x.MinimumNumCores,
            Tags = x.Tags.ToList()
        };
    }

    public static ActionModel ToAction(this ActionResponse x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            ActionPriority = x.Priority,
            Order = x.Order,
            MinimumRam = x.MinimumRam,
            MinimumNumCores = x.MinimumNumCores,
            Tags = x.Tags.ToList()
        };
    }

    public static CloudModel ToCloud(this CloudRequest x)
    {
        return new()
        {
            Name = x.Name,
            CloudIp = x.IpAddress,
            MacAddress = x.MacAddress,
            CloudStatus = x.Status,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            LastUpdatedTime = DateTime.Now,
            Organization = x.Organization
        };
    }

    public static CloudModel ToCloud(this CloudResponse x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            CloudIp = x.IpAddress,
            MacAddress = x.MacAddress,
            CloudStatus = x.Status,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            LastUpdatedTime = x.LastUpdatedTime
        };
    }

    public static List<CloudModel> ToCloudList(this GetCloudsResponse clouds)
    {
        return clouds.Clouds.Select(x =>
            new CloudModel
            {
                Id = x.Id,
                Name = x.Name,
                CloudIp = x.IpAddress,
                MacAddress = x.MacAddress,
                CloudStatus = x.Status,
                Cpu = x.Cpu,
                NumberOfCores = x.NumberOfCores,
                Ram = x.Ram,
                VirtualRam = x.VirtualRam,
                DiskStorage = x.DiskStorage,
                LastUpdatedTime = x.LastUpdatedTime
            }).ToList();
    }

    public static ContainerImageModel ToContainer(this ContainerRequest x)
    {
        return new()
        {
            Name = x.Name,
            Description = x.Description,
            K8SDeployment = x.K8SDeployment,
            K8SService = x.K8SService,
            Timestamp = DateTime.Now
        };
    }

    public static ContainerImageModel ToContainer(this ContainerResponse x)
    {
        return new()
        {
            Name = x.Name,
            Description = x.Description,
            K8SDeployment = x.K8sDeployment,
            K8SService = x.K8sService,
            Timestamp = DateTime.Now
        };
    }

    public static List<ContainerImageModel> ToContainersList(this GetContainersResponse containers)
    {
        return containers.Containers.Select(x =>
            new ContainerImageModel
            {
                Id = x.Id,
                Name = x.Name,
                Description = x.Description,
                K8SDeployment = x.K8sDeployment,
                K8SService = x.K8sService,
                Timestamp = x.LastUpdateTime
            }).ToList();
    }

    public static EdgeModel ToEdge(this EdgeRequest x)
    {
        return new()
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
            LastUpdatedTime = DateTime.Now,
            Organization = x.Organization
        };
    }

    public static EdgeModel ToEdge(this EdgeResponse x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            EdgeStatus = x.Status,
            EdgeIp = x.IpAddress,
            MacAddress = x.MacAddress,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            LastUpdatedTime = x.LastUpdatedTime
        };
    }

    public static List<EdgeModel> ToEdgeList(this GetEdgesResponse edges)
    {
        return edges.Edges.Select(x => new EdgeModel
        {
            Id = x.Id,
            Name = x.Name,
            EdgeStatus = x.Status,
            EdgeIp = x.IpAddress,
            MacAddress = x.MacAddress,
            Cpu = x.Cpu,
            NumberOfCores = x.NumberOfCores,
            Ram = x.Ram,
            VirtualRam = x.VirtualRam,
            DiskStorage = x.DiskStorage,
            LastUpdatedTime = x.LastUpdatedTime
        }).ToList();
    }

    public static InstanceModel ToInstance(this InstanceRequest x)
    {
        return new()
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

    public static InstanceModel ToInstance(this InstanceResponse x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            InstanceFamily = x.Family,
            ServiceType = x.Type,
            IsReusable = x.IsReusable,
            MinimumRam = x.MinimumRam,
            MinimumNumCores = x.MinimumNumOfCores,
            RosVersion = x.RosVersion,
            ROSDistro = x.RosDistro,
            RosTopicsPub = x.RosTopicPublishers.ToList(),
            RosTopicsSub = x.RosTopicSubscribers.ToList(),
            Tags = x.Tags?.ToList(),
            OnboardedTime = x.OnboardedTime
        };
    }

    public static PolicyModel ToPolicy(this PolicyRequest x)
    {
        return new()
        {
            Name = x.Name,
            Description = x.Description,
            Type = x.Type,
            IsActive = x.IsActive,
            IsExclusiveWithinType = x.IsExclusiveWithinType,
            Timestamp = x.LastTimeUpdated
        };
    }

    public static PolicyModel ToPolicy(this PolicyResponse x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Description = x.Description,
            Type = x.Type,
            IsActive = x.IsActive,
            IsExclusiveWithinType = x.IsExclusiveWithinType,
            Timestamp = x.LastTimeUpdated
        };
    }

    public static List<PolicyModel> ToPolicyList(this GetPoliciesResponse policies)
    {
        return policies.Policies.Select(x =>
            new PolicyModel
            {
                Id = x.Id,
                Name = x.Name,
                Description = x.Description,
                Type = x.Type,
                IsActive = x.IsActive,
                IsExclusiveWithinType = x.IsExclusiveWithinType,
                Timestamp = x.LastTimeUpdated
            }).ToList();
    }

    public static RobotModel ToRobot(this RobotRequest x)
    {
        return new()
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
            LastUpdatedTime = DateTime.Now,
            SimCardNumber = x.SimCardNumber
        };
    }

    public static RobotModel ToRobot(this RobotResponse x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            RobotModelName = x.ModelName,
            RobotStatus = x.Status,
            BatteryStatus = x.BatteryStatus,
            RosVersion = x.RosVersion,
            RosDistro = x.RosDistro,
            ROSRepo = x.RosRepo,
            ROSNodes = x.RosNodes?.ToList(),
            MaximumPayload = x.MaximumPayload,
            MaximumTranslationalVelocity = x.MaximumTranslationalVelocity,
            MaximumRotationalVelocity = x.MaximumRotationalVelocity,
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
            LastUpdatedTime = x.LastUpdatedTime,
            OnboardedTime = x.OnboardedTime,
            SimCardNumber = x.SimCardNumber
        };
    }

    public static TaskModel ToTask(this TaskRequest x)
    {
        return new()
        {
            Name = x.Name,
            TaskPriority = (int)Enum.Parse<Priority>(x.Priority, true),
            DeterministicTask = x.IsDeterministic,
            Tags = x.Tags?.ToList() ?? new List<string>()
        };
    }

    public static TaskModel ToTask(this TaskResponse x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            DeterministicTask = x.IsDeterministic,
            TaskPriority = (int)Enum.Parse<Priority>(x.Priority),
            Tags = x.Tags?.ToList() ?? new List<string>()
        };
    }

    public static List<SliceModel> ToSliceList(this RegisterSlicesRequest x)
    {
        return x.Slices.Select(s => new SliceModel
        {
            Name = s.SliceId,
            Site = s.Site,
            ExpDataRateDl = s.ExpDataRateDl,
            ExpDataRateUl = s.ExpDataRateUl,
            Jitter = s.Jitter,
            Latency = s.Latency,
            UserDensity = s.UserDensity,
            UserSpeed = s.UserSpeed,
            TrafficType = Enum.Parse<TrafficType>(s.TrafficType, true),
            Imsi = s.Imsi.ToList()
        }).ToList();
    }

    public static Location ToLocation(this LocationRequest x)
    {
        return new()
        {
            Id = x.Id,
            Name = x.Name,
            Organization = x.Organization,
            Type = Enum.Parse<LocationType>(x.Type)
        };
    }
}