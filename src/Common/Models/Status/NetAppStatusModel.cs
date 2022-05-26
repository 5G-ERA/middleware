namespace Middleware.Common.Models;

public class NetAppStatusModel : BaseModel
{
    /// <summary>
    /// Identifier of the NetApp
    /// </summary>
    public override Guid Id { get; set; }
    /// <summary>
    /// Name of the NetApp
    /// </summary>
    public override string Name { get; set; }
    /// <summary>
    /// Hard limit of how many robots the NetApp can work for
    /// </summary>
    public int HardLimit { get; set; }
    /// <summary>
    /// Soft limit of how many robots the NetApp can work for, it represents optimal number of robots
    /// </summary>
    public int OptimalLimit { get; set; }
    /// <summary>
    /// Number of robots the NetApp currently works for
    /// </summary>
    public int CurrentRobotsCount { get; set; }
}