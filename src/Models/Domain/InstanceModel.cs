using Middleware.Models.Domain.Contracts;
using Middleware.Models.Domain.Ros;
using Middleware.Models.Domain.ValueObjects;
using Middleware.Models.Dto;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class InstanceModel : BaseModel, IPolicyAssignable, IHardwareRequirementClaim
{
    public override Guid Id { get; set; } = Guid.NewGuid();

    public override string Name { get; set; } = default!; // compulsory field

    public Guid ServiceInstanceId { get; set; }

    public string? ServiceType { get; set; }

    public bool? IsReusable { get; set; }

    public string? DesiredStatus { get; set; }

    public string? ServiceUrl { get; set; }

    public List<RosTopicModel> RosTopicsPub { get; set; } = new(); // compulsory field

    public List<RosTopicModel> RosTopicsSub { get; set; } = new(); // compulsory field

    /// <summary>
    ///     Represents list of transformations made when connecting to relay from robot
    /// </summary>
    public List<RosTransformsModel> TransformsToNetApp { get; set; } = new();

    public List<RosTransformsModel> Actions { get; set; } = new();
    public List<RosTransformsModel> Transforms { get; set; } = new();

    public int RosVersion { get; set; } // compulsory field

    public string? RosDistro { get; set; } // compulsory field

    public List<string>? Tags { get; set; }

    public string? InstanceFamily { get; set; } // Compulsory field

    public int SuccessRate { get; set; }

    public string? ServiceStatus { get; set; }
    public ContainerImageModel? ContainerImage { get; set; }
    public DateTime OnboardedTime { get; set; } // Compulsory field

    public DateTimeOffset? LastStatusChange { get; set; }

    /// <inheritdoc />
    public List<string> AppliedPolicies { get; init; } = new();

    /// <summary>
    ///     Set the new operation status for the Instance
    /// </summary>
    /// <param name="status"></param>
    public void SetStatus(ServiceStatus status)
    {
        ServiceStatus = status.ToString();
        LastStatusChange = DateTimeOffset.UtcNow;
    }

    /// <summary>
    ///     Indicates if the instance is considered terminated / down
    /// </summary>
    /// <returns></returns>
    public bool CanBeDeleted()
    {
        return ServiceStatus == Enums.ServiceStatus.Down.ToString()
               // TODO: extract the time comparison to external service
               && DateTimeOffset.Now - LastStatusChange > new TimeSpan(1, 0, 0);
    }


    /// <summary>
    ///     Can the instance be reused by multiple consumers
    /// </summary>
    /// <returns></returns>
    public bool CanBeReused()
    {
        return IsReusable != null && IsReusable.Value;
    }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new InstanceDto
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            ServiceInstanceId = domain.ServiceInstanceId.ToString(),
            ServiceType = domain.ServiceType,
            IsReusable = domain.IsReusable,
            DesiredStatus = domain.DesiredStatus,
            ServiceUrl = domain.ServiceUrl,
            RosTopicsPub = domain.RosTopicsPub.Select(x => x.ToDto()).ToList(),
            RosTopicsSub = domain.RosTopicsSub.Select(x => x.ToDto()).ToList(),
            RosVersion = domain.RosVersion,
            ROSDistro = domain.RosDistro,
            Tags = domain.Tags ?? new List<string>(),
            InstanceFamily = domain.InstanceFamily,
            SuccessRate = domain.SuccessRate,
            ServiceStatus = domain.ServiceStatus,
            HardwareRequirements = new()
            {
                MinimumRam = domain.Ram?.Minimal,
                OptimalRam = domain.Ram?.Optimal,
                RamPriority = domain.Ram?.Priority.ToString(),
                MinimumNumberOfCores = domain.NumberOfCores?.Minimal,
                OptimalNumberOfCores = domain.NumberOfCores?.Optimal,
                NumberOfCoresPriority = domain.NumberOfCores?.Priority.ToString(),
                MinimumDiskStorage = domain.DiskStorage?.Minimal,
                OptimalDiskStorage = domain.DiskStorage?.Optimal,
                DiskStoragePriority = domain.DiskStorage?.Priority.ToString(),
                MinimumThroughput = domain.Throughput?.Minimal,
                OptimalThroughput = domain.Throughput?.Optimal,
                ThroughputPriority = domain.Throughput?.Priority.ToString(),
                MinimumLatency = domain.Latency?.Minimal,
                OptimalLatency = domain.Latency?.Optimal,
                LatencyPriority = domain.Latency?.Priority.ToString()
            },
            OnboardedTime = domain.OnboardedTime == default ? DateTimeOffset.Now : domain.OnboardedTime,
            LastStatusChange = domain.LastStatusChange,
            AppliedPolicies = domain.AppliedPolicies
        };
    }

    public void SetNetAppAddress(string netAppAddress)
    {
        if (!Uri.IsWellFormedUriString(netAppAddress, UriKind.Absolute))
            throw new ArgumentException("Specified NetApp address is not a valid Url string", nameof(netAppAddress));

        ServiceUrl = netAppAddress;
    }

    #region Resource Reguirements

    public NetAppRequirement? Ram { get; set; }

    public NetAppRequirement? NumberOfCores { get; set; }
    public NetAppRequirement? DiskStorage { get; set; }
    public NetAppRequirement? Throughput { get; set; }
    public NetAppRequirement? Latency { get; set; }

    #endregion
}