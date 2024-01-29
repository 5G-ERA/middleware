using System.ComponentModel.DataAnnotations;
using Middleware.Models.Dto;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class NetAppStatusModel : BaseModel
{
    /// <summary>
    ///     Identifier of the NetApp
    /// </summary>
    public override Guid Id { get; set; }

    /// <summary>
    ///     Name of the NetApp
    /// </summary>
    public override string Name { get; set; } = default!;

    /// <summary>
    ///     Hard limit of how many robots the NetApp can work for
    /// </summary>
    public int HardLimit { get; set; }

    /// <summary>
    ///     Soft limit of how many robots the NetApp can work for, it represents optimal number of robots
    /// </summary>
    public int OptimalLimit { get; set; }

    /// <summary>
    ///     Number of robots the NetApp currently works for
    /// </summary>
    [Required]
    public int? CurrentRobotsCount { get; set; }

    /// <summary>
    ///     Timestamp of the update
    /// </summary>
    public DateTimeOffset Timestamp { get; set; }

    public string Colour
    {
        get { return GetColourCodedStatus().ToString().ToLower(); }
    }

    public ColourCode GetColourCodedStatus()
    {
        var retVal = ColourCode.Yellow;
        if (CurrentRobotsCount < OptimalLimit)
        {
            retVal = ColourCode.Green;
        }

        if (CurrentRobotsCount >= HardLimit)
        {
            retVal = ColourCode.Red;
        }

        return retVal;
    }


    public bool IsValid()
    {
        return Id != Guid.Empty && HardLimit != default && OptimalLimit != default
               && CurrentRobotsCount.HasValue;
    }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new NetAppStatusDto
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            HardLimit = domain.HardLimit,
            OptimalLimit = domain.OptimalLimit,
            CurrentRobotsCount = domain.CurrentRobotsCount,
            Timestamp = domain.Timestamp
        };
    }
}