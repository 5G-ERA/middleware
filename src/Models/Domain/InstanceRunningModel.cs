using System.Text.Json.Serialization;
using Middleware.Models.Dto;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Enums;


namespace Middleware.Models.Domain;

public class InstanceRunningModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; }

    [JsonPropertyName("ServiceInstanceId")]
    public Guid ServiceInstanceId { get; set; }

    [JsonPropertyName("ServiceType")]

    public string ServiceType { get; set; }

    [JsonPropertyName("ServiceUrl")]
    public string ServiceUrl { get; set; }

    [JsonPropertyName("ServiceStatus")]
    public string ServiceStatus { get; set; } //updated every 10 sec

    [JsonPropertyName("DeployedTime")]
    public DateTime DeployedTime { get; set; } // Compulsory field

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new InstanceRunningDto()
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            ServiceType = domain.ServiceType,
            ServiceInstanceId = domain.ServiceInstanceId.ToString(),
            ServiceUrl = domain.ServiceUrl,
            ServiceStatus = domain.ServiceStatus,
            DeployedTime = domain.DeployedTime == default ? DateTimeOffset.Now : domain.DeployedTime
        };
    }
}
