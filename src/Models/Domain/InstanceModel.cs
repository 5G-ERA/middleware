using Middleware.Models.Domain.Contracts;
using Middleware.Models.Dto;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class InstanceModel : BaseModel, IPolicyAssignable
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

    public int RosVersion { get; set; } // compulsory field

    public string? RosDistro { get; set; } // compulsory field

    public List<string>? Tags { get; set; }

    public string? InstanceFamily { get; set; } // Compulsory field

    public int SuccessRate { get; set; }

    public string? ServiceStatus { get; internal set; }
    public ContainerImageModel? ContainerImage { get; set; }

    public long? MinimumRam { get; set; } // Compulsory field

    public int? MinimumNumCores { get; set; } // Compulsory field

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
               && DateTimeOffset.Now - LastStatusChange > new TimeSpan(1, 0, 0);
    }


    /// <summary>
    ///     On boarding validation of the instance data object.
    /// </summary>
    /// <returns>bool</returns>
    public bool IsValid()
    {
        var rosDistroNames = Enum.GetNames(typeof(RosDistro)).ToList();

        //if (string.IsNullOrWhiteSpace(Name)) return false;
        if (RosVersion > 2) return false;
        if (RosVersion == 0) return false;
        if (string.IsNullOrEmpty(MinimumRam.ToString())) return false;
        if (string.IsNullOrEmpty(MinimumNumCores.ToString())) return false;
        if (string.IsNullOrEmpty(RosDistro)) return false;
        if (string.IsNullOrEmpty(InstanceFamily)) return false;
        if (!rosDistroNames.Contains(RosDistro)) return false;

        return true;
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
                MinimumRam = domain.MinimumRam,
                MinimumNumCores = domain.MinimumNumCores
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
}