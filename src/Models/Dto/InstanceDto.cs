using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Dto.Ros;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "instance-idx", StorageType = StorageType.Json, Prefixes = new[] { Prefix })]
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
    public string? ServiceType { get; set; } = default!;

    [Indexed]
    public bool? IsReusable { get; set; } = default!;

    [Indexed]
    public bool IsPersistent { get; set; } = default!;
    
    [Indexed]
    public string? DesiredStatus { get; set; } = default!;

    [Indexed]
    public string? ServiceUrl { get; set; } = default!;

    [Indexed]
    public List<RosTopic> RosTopicsPub { get; set; } = new();

    [Indexed]
    public List<RosTopic> RosTopicsSub { get; set; } = new();
    [Indexed]
    public List<RosAction> Actions { get; set; } = new();
    [Indexed]
    public List<RosService> Services { get; set; } = new();
    [Indexed]
    public List<RosTransforms> Transforms { get; set; } = new();

    [Indexed(Sortable = true)]
    public int RosVersion { get; set; } = default!;

    [Indexed]
    public string? ROSDistro { get; set; } = default!;

    [Indexed]
    public List<string> Tags { get; set; } = new();

    [Indexed]
    public string? InstanceFamily { get; set; } = default!;

    [Indexed(Sortable = true)]
    public int SuccessRate { get; set; } = default!;

    [Indexed]
    public string? ServiceStatus { get; set; } = default!;

    [Indexed]
    public HardwareRequirements HardwareRequirements { get; set; } = new();

    [Indexed(Sortable = true)]
    public DateTimeOffset OnboardedTime { get; set; }

    public DateTimeOffset? LastStatusChange { get; set; }
    public List<string> AppliedPolicies { get; init; } = new();

    public override BaseModel ToModel()
    {
        var dto = this;
        return new InstanceModel
        {
            Id = Guid.Parse(dto.Id.Replace(Prefix, "")),
            Name = dto.Name!,
            ServiceInstanceId = string.IsNullOrEmpty(dto.ServiceInstanceId)
                ? Guid.Empty
                : Guid.Parse(dto.ServiceInstanceId),
            ServiceType = dto.ServiceType,
            IsReusable = dto.IsReusable,
            IsPersistent = dto.IsPersistent,
            DesiredStatus = dto.DesiredStatus,
            ServiceUrl = dto.ServiceUrl,
            RosTopicsPub = dto.RosTopicsPub.Select(x => x.ToModel()).ToList(),
            RosTopicsSub = dto.RosTopicsSub.Select(x => x.ToModel()).ToList(),
            Actions = dto.Actions.Select(x => x.ToModel()).ToList(),
            Services = dto.Services.Select(x=>x.ToModel()).ToList(),
            Transforms = dto.Transforms.Select(t=>t.ToModel()).ToList(),
            RosVersion = dto.RosVersion,
            RosDistro = dto.ROSDistro,
            Tags = dto.Tags,
            InstanceFamily = dto.InstanceFamily,
            SuccessRate = dto.SuccessRate,
            ServiceStatus = dto.ServiceStatus,
            Ram = new(dto.HardwareRequirements.MinimumRam, dto.HardwareRequirements.OptimalRam,
                dto.HardwareRequirements.RamPriority),
            NumberOfCores = new(dto.HardwareRequirements.MinimumNumberOfCores,
                dto.HardwareRequirements.OptimalNumberOfCores, dto.HardwareRequirements.NumberOfCoresPriority),
            DiskStorage = new(dto.HardwareRequirements.MinimumDiskStorage, dto.HardwareRequirements.OptimalDiskStorage,
                dto.HardwareRequirements.DiskStoragePriority),
            Throughput = new(dto.HardwareRequirements.MinimumThroughput, dto.HardwareRequirements.OptimalThroughput,
                dto.HardwareRequirements.ThroughputPriority),
            Latency = new(dto.HardwareRequirements.MinimumLatency, dto.HardwareRequirements.OptimalLatency,
                dto.HardwareRequirements.LatencyPriority),
            OnboardedTime = dto.OnboardedTime.DateTime,
            LastStatusChange = dto.LastStatusChange,
            AppliedPolicies = dto.AppliedPolicies
        };
    }
}