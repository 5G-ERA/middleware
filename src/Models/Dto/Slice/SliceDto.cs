using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Slice;

[Document(IndexName = "slice-idx", StorageType = StorageType.Json, Prefixes = new[] { Prefix })]
public class SliceDto : Dto
{
    public const string Prefix = "Slice";

    [Indexed] [RedisIdField] public override string Id { get; set; } = default!;

    [Indexed] public string Name { get; set; } = default!;

    [Indexed] public string Site { get; init; } = default!;

    [Indexed] public string SliceType { get; init; } = default!;

    [Indexed] public int ExpDataRateUl { get; init; }

    [Indexed] public int ExpDataRateDl { get; init; }

    [Indexed] public int? Latency { get; init; }

    [Indexed] public int? Jitter { get; init; }

    [Indexed] public int? UserDensity { get; init; }

    [Indexed] public int? UserSpeed { get; init; }

    [Indexed] public string TrafficType { get; init; } = default!;

    [Indexed] public List<string> Imsi { get; init; } = new();

    public override BaseModel ToModel()
    {
        var dto = this;
        return new SliceModel
        {
            Id = Guid.Parse(dto.Id),
            Name = dto.Name,
            Site = dto.Site,
            ExpDataRateUl = dto.ExpDataRateUl,
            ExpDataRateDl = dto.ExpDataRateDl,
            Latency = dto.Latency,
            Jitter = dto.Jitter,
            UserDensity = dto.UserDensity,
            UserSpeed = dto.UserSpeed,
            TrafficType = Enum.Parse<TrafficType>(dto.TrafficType, true),
            Imsi = dto.Imsi
        };
    }
}