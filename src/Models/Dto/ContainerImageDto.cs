using Middleware.Models.Domain;
using Redis.OM.Modeling;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Models.Dto;
[Document(IndexName = "containerImage-idx", StorageType = StorageType.Json, Prefixes = new[] { "ContainerImage" })]
public class ContainerImageDto : Dto
{
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;
    [Indexed]
    public string Name { get; set; } = default!;
    [Indexed(Sortable = true)]
    public DateTime Timestamp { get; set; } = default!;
    [Indexed]
    public string Description { get; set; } = default!;
    [Indexed]
    public string K8SDeployment { get; set; } = default!;
    [Indexed]
    public string K8SService { get; set; } = default!;

    public override BaseModel ToModel()
    {
        var dto = this;
        return new ContainerImageModel()
        {
            Id = Guid.Parse(dto.Id!),
            Name = dto.Name,
            Timestamp = dto.Timestamp,
            Description = dto.Description,
            K8SDeployment = dto.K8SDeployment,
            K8SService = dto.K8SService
        };
    }

}
