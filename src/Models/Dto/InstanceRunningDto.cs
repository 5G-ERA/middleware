using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Redis.OM.Modeling;
using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Dto;

[Document(IndexName = "instanceRunning-idx", StorageType = StorageType.Json, Prefixes = new[] { InstanceRunningDto.Prefix })]
public class InstanceRunningDto : Dto
{
    public const string Prefix = "instanceRunning";
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
    public string ServiceUrl { get; set; } = default!;
    [Indexed]
    public string ServiceStatus { get; set; } = default!;

    [Indexed(Sortable = true)]
    public DateTimeOffset DeployedTime { get; set; }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new InstanceRunningModel()
        {
            Id = Guid.Parse(dto.Id.Replace(Prefix, "")),
            Name = dto.Name,
            ServiceType = dto.ServiceType,
            ServiceUrl = dto.ServiceUrl,
            ServiceInstanceId = Guid.Parse(dto.ServiceInstanceId),
            ServiceStatus = dto.ServiceStatus,
            DeployedTime = dto.DeployedTime.DateTime

        };
    }
}
