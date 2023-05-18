using Middleware.Models.Enums;

namespace Middleware.ResourcePlanner.Policies;

internal interface IPolicy
{
    /// <summary>
    ///     Configured priority for the Policy
    /// </summary>
    Priority Priority { get; }
}