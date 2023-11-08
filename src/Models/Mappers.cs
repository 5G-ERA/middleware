using Middleware.Models.Domain;
using Middleware.Models.Enums;

namespace Middleware.Models;

public static class Mappers
{
    public static Location ToLocation(this CloudModel x)
    {
        return new()
        {
            Id = x.Id,
            Type = LocationType.Cloud,
            Name = x.Name,
            Address = x.Address,
            Cpu = x.Cpu,
            DiskStorage = x.DiskStorage,
            IsOnline = x.IsOnline,
            LastUpdatedTime = x.LastUpdatedTime,
            MacAddress = x.MacAddress,
            NumberOfCores = x.NumberOfCores,
            Organization = x.Organization,
            Ram = x.Ram,
            Status = x.CloudStatus,
            VirtualRam = x.VirtualRam
        };
    }

    public static List<Location> ToLocations(this IEnumerable<CloudModel> x)
    {
        return x.Select(c => c.ToLocation()).ToList();
    }
}