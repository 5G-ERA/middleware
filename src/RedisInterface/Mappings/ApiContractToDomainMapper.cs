using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Mappings;

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
    public static CloudModel ToCloud(this UpdateCloudRequest x)
    {
        return new CloudModel()
        {
            Id = x.Id,
            Name = x.Cloud.Name,
            Type = x.Cloud.Type,
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
}