using Middleware.Models.Domain.ValueObjects;

namespace Middleware.Models.Domain.Contracts;

public interface IHardwareRequirementClaim
{
    public NetAppRequirement? Ram { get; set; }

    public NetAppRequirement? NumberOfCores { get; set; }
    public NetAppRequirement? DiskStorage { get; set; }
    public NetAppRequirement? Throughput { get; set; }
    public NetAppRequirement? Latency { get; set; }
}