using Middleware.Models.Domain;
using Redis.OM.Modeling;
using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Dto;

[Document(IndexName = "instance-idx", StorageType = StorageType.Json, Prefixes = new[] { "Instance" })]
public class InstanceDto : Dto
{
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
    public ContainerImageDto ContainerImage { get; set; } = new();
    [Indexed(Sortable = true)]
    public int MinimumRam { get; set; } = default!;
    [Indexed(Sortable = true)]
    public int MinimumNumCores { get; set; } = default!;

    public override BaseModel ToModel()
    {
        var dto = this;
        return new InstanceModel()
        {
            Id = Guid.Parse(dto.Id),
            Name = dto.Name,
            ServiceInstanceId = Guid.Parse(dto.ServiceInstanceId),
            ServiceType = dto.ServiceType,
            IsReusable = dto.IsReusable,
            DesiredStatus = dto.DesiredStatus,
            ServiceUrl = dto.ServiceUrl,
            RosTopicsPub = dto.RosTopicsPub.ToList().Select(x => x.ToModel()).ToList(),

        };
    }
}
