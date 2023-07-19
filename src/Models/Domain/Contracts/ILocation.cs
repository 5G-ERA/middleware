using Middleware.Models.Enums;

namespace Middleware.Models.Domain.Contracts;

public interface ILocation
{
    /// <summary>
    ///     Id of the location
    /// </summary>
    Guid Id { get; }

    /// <summary>
    ///     Name of teh Organization the Location belongs to
    /// </summary>
    string Organization { get; }

    /// <summary>
    ///     Address of the Location
    /// </summary>
    Uri Address { get; }

    /// <summary>
    ///     Name of the location
    /// </summary>
    string Name { get; }

    /// <summary>
    ///     Type of the location
    /// </summary>
    LocationType Type { get; }

    /// <summary>
    ///     Inheritant of <seealso cref="BaseModel" /> that represents Location
    /// </summary>
    /// <returns></returns>
    BaseModel ToBaseLocation();

    /// <summary>
    ///     Location representation of the
    /// </summary>
    /// <returns></returns>
    Location ToLocation();

    /// <summary>
    ///     Prepares the route that will be used to communicate with the NetApp through Gateway in the location
    /// </summary>
    /// <param name="netAppName"></param>
    /// <returns></returns>
    string GetNetAppAddress(string netAppName);

    string GetNetAppStatusReportAddress();
}