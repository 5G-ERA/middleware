using Middleware.Models.Dto.Slice;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain.Slice;

public class SliceModel
{
    /// <summary>
    ///     Unique identifier of a Slice
    /// </summary>
    public string SliceId { get; set; } = default!;

    /// <summary>
    ///     Site the Slice is located
    /// </summary>
    public string Site { get; set; } = default!;

    /// <summary>
    ///     Type of slice, either Urllc or Embb
    /// </summary>
    public SliceType SliceType
    {
        get
        {
            if (Latency is not null && Jitter is not null)
                return SliceType.Urllc;

            return SliceType.Embb;
        }
    }

    /// <summary>
    ///     Expected Up Link of a Slice
    /// </summary>
    public int ExpDataRateUl { get; set; }

    /// <summary>
    ///     Expected Down Link of a Slice
    /// </summary>
    public int ExpDataRateDl { get; set; }

    /// <summary>
    ///     Expected Latency, only in Urllc Slice
    /// </summary>
    public int? Latency { get; set; }

    /// <summary>
    ///     Expected Jitter, only in Urllc Slice
    /// </summary>
    public int? Jitter { get; set; }

    /// <summary>
    ///     Configured expected number of users, only in Embb Slice
    /// </summary>
    public int? UserDensity { get; set; }

    /// <summary>
    ///     Configured single user speed, only in Urllc Slice
    /// </summary>
    public int? UserSpeed { get; set; }

    /// <summary>
    ///     Traffic Type supported by a Slice, either TCP or UDP
    /// </summary>
    public TrafficType TrafficType { get; set; }

    /// <summary>
    ///     List of the SIM Card identification numbers associated with a Slice
    /// </summary>
    public List<string> Imsi { get; set; } = new();

    public SliceDto ToDto()
    {
        var d = this;
        return new()
        {
            SliceId = d.SliceId,
            Site = d.Site,
            SliceType = d.SliceType.ToString(),
            ExpDataRateUl = d.ExpDataRateUl,
            ExpDataRateDl = d.ExpDataRateDl,
            Latency = d.Latency,
            Jitter = d.Jitter,
            UserDensity = d.UserDensity,
            UserSpeed = d.UserSpeed,
            TrafficType = d.TrafficType.ToString(),
            Imsi = d.Imsi
        };
    }
}