using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Policies.Contracts;


/// <summary>
/// Abstract class for implementing Location specific policies
/// </summary>
internal abstract class AbstractLocationPolicy
{
    public abstract Location GetLocation();
}