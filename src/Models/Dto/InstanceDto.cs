using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Redis.OM.Modeling;
using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Dto;

[Document(IndexName = "instance-idx", StorageType = StorageType.Json, Prefixes = new[] { InstanceDto.Prefix })]
public class InstanceDto : Dto
{
    public const string Prefix = "Instance";
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;
    [Indexed]
    public string? Name { get; set; } = default!;
    [Indexed]
    public string? ServiceInstanceId { get; set; } = default!;
    [Indexed]
    public string ServiceType { get; set; } = default!;
    [Indexed]
    public bool? IsReusable { get; set; } = default!;
    [Indexed]
    public string DesiredStatus { get; set; } = default!;
    [Indexed]
    public string ServiceUrl { get; set; } = default!;
    [Indexed]
    public List<RosTopic> RosTopicsPub { get; set; } = new();
    [Indexed]
    public List<RosTopic> RosTopicsSub { get; set; } = new();
    [Indexed(Sortable = true)]
    public int RosVersion { get; set; } = default!;
    [Indexed]
    public string ROSDistro { get; set; } = default!;
    [Indexed]
    public List<string> Tags { get; set; } = new();
    [Indexed]
    public string InstanceFamily { get; set; } = default!;
    [Indexed(Sortable = true)]
    public int SuccessRate { get; set; } = default!;
    [Indexed]
    public string ServiceStatus { get; set; } = default!;
    [Indexed]
    public HardwareRequirements HardwareRequirements { get; set; } = new();

    [Indexed(Sortable = true)]
    public DateTimeOffset OnboardedTime { get; set; }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new InstanceModel()
        {
            Id = Guid.Parse(dto.Id.Replace(Prefix, "")),
            Name = dto.Name,
            ServiceInstanceId = Guid.Parse(dto.ServiceInstanceId),
            ServiceType = dto.ServiceType,
            IsReusable = dto.IsReusable,
            DesiredStatus = dto.DesiredStatus,
            ServiceUrl = dto.ServiceUrl,
            RosTopicsPub = dto.RosTopicsPub.Select(x => x.ToModel()).ToList(),
            RosTopicsSub = dto.RosTopicsSub.Select(x => x.ToModel()).ToList(),
            RosVersion = dto.RosVersion,
            RosDistro = dto.ROSDistro,
            Tags = dto.Tags,
            InstanceFamily = dto.InstanceFamily,
            SuccessRate = dto.SuccessRate,
            ServiceStatus = dto.ServiceStatus,
            MinimumRam = dto.HardwareRequirements.MinimumRam,
            MinimumNumCores = dto.HardwareRequirements.MinimumNumCores,
            OnboardedTime = dto.OnboardedTime.DateTime
        };
    }
}
