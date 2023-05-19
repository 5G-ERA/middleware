using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;

namespace Middleware.Models.Dto.Slice;

public class SliceDto
{
    public string SliceId { get; init; } = default!;
    public string Site { get; init; } = default!;
    public string SliceType { get; init; } = default!;
    public int ExpDataRateUl { get; init; }
    public int ExpDataRateDl { get; init; }
    public int? Latency { get; init; }
    public int? Jitter { get; init; }
    public int? UserDensity { get; init; }
    public int? UserSpeed { get; init; }
    public string TrafficType { get; init; } = default!;
    public List<string> Imsi { get; init; } = new();

    public SliceModel ToModel()
    {
        var dto = this;
        return new()
        {
            SliceId = dto.SliceId,
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