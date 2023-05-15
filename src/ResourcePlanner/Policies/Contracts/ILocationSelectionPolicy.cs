using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Policies;

internal interface ILocationSelectionPolicy : IPolicy
{
    Task<Location> GetLocationAsync();
}