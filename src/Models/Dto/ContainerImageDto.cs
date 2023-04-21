using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;
[Document(IndexName = "containerImage-idx", StorageType = StorageType.Json, Prefixes = new[] { ContainerImageDto.Prefix })]
public class ContainerImageDto : Dto
{
    public const string Prefix = "ContainerImage";
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;
    [Indexed]
    public string Name { get; set; } = default!;
    [Indexed(Sortable = true)]
    public DateTimeOffset Timestamp { get; set; } = DateTimeOffset.Now;
    [Indexed]
    public string? Description { get; set; } = default!;
    [Indexed]
    public string K8SDeployment { get; set; } = default!;
    [Indexed]
    public string? K8SService { get; set; } = default!;

    public override ContainerImageModel ToModel()
    {
        var dto = this;
        return new ContainerImageModel()
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name,
            Timestamp = dto.Timestamp.DateTime,
            Description = dto.Description,
            K8SDeployment = dto.K8SDeployment,
            K8SService = dto.K8SService
        };
    }

}
