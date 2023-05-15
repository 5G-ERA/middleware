using System.Text.Json.Serialization;
using Middleware.Models.Dto;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class InstanceModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; } = Guid.NewGuid();

    [JsonPropertyName("Name")]
    public override string? Name { get; set; } // compulsory field
    
    [JsonPropertyName("ServiceInstanceId")]
    public Guid ServiceInstanceId { get; set; }

    [JsonPropertyName("ServiceType")]
    public string? ServiceType { get; set; }

    [JsonPropertyName("IsReusable")]
    public bool? IsReusable { get; set; }

    [JsonPropertyName("DesiredStatus")]
    public string? DesiredStatus { get; set; }

    [JsonPropertyName("ServiceUrl")]
    public string? ServiceUrl { get; set; }

    [JsonPropertyName("RosTopicsPub")]
    public List<RosTopicModel> RosTopicsPub { get; set; } = new();// compulsory field

    [JsonPropertyName("RosTopicsSub")]
    public List<RosTopicModel> RosTopicsSub { get; set; } = new();// compulsory field

    [JsonPropertyName("ROSVersion")]
    public int RosVersion { get; set; } // compulsory field

    [JsonPropertyName("ROSDistro")]
    public string? RosDistro { get; set; } // compulsory field

    [JsonPropertyName("Tags")]
    public List<string>? Tags { get; set; }

    [JsonPropertyName("InstanceFamily")]
    public string? InstanceFamily { get; set; } // Compulsory field

    [JsonPropertyName("SuccessRate")]
    public int SuccessRate { get; set; }

    [JsonPropertyName("ServiceStatus")]
    public string? ServiceStatus { get; set; } //updated every 10 sec

    [JsonPropertyName("ContainerImage")]
    [JsonIgnore]
    public ContainerImageModel? ContainerImage { get; set; }

    [JsonPropertyName("MinimumRam")]
    public long? MinimumRam { get; set; } // Compulsory field

    [JsonPropertyName("MinimumNumCores")]
    public int? MinimumNumCores { get; set; } // Compulsory field

    [JsonPropertyName("OnboardedTime")]
    public DateTime OnboardedTime { get; set; } // Compulsory field

    /// <summary>
    /// On boarding validation of the instance data object.
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
    /// Can the instance be reused by multiple consumers
    /// </summary>
    /// <returns></returns>
    public bool CanBeReused()
    {
        return IsReusable != null && IsReusable.Value;
    }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new InstanceDto()
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            ServiceInstanceId = domain.ServiceInstanceId.ToString(),
            ServiceType = domain.ServiceType,
            IsReusable = domain.IsReusable,
            DesiredStatus = domain.DesiredStatus,
            ServiceUrl = domain.ServiceUrl,
            RosTopicsPub = domain.RosTopicsPub?.Select(x => x.ToDto()).ToList(),
            RosTopicsSub = domain.RosTopicsSub?.Select(x => x.ToDto()).ToList(),
            RosVersion = domain.RosVersion,
            ROSDistro = domain.RosDistro,
            Tags = domain.Tags,
            InstanceFamily = domain.InstanceFamily,
            SuccessRate = domain.SuccessRate,
            ServiceStatus = domain.ServiceStatus,
            HardwareRequirements = new HardwareRequirements()
            {
                MinimumRam = domain.MinimumRam,
                MinimumNumCores = domain.MinimumNumCores
            },
            OnboardedTime = domain.OnboardedTime == default ? DateTimeOffset.Now : domain.OnboardedTime
        };
    }
}