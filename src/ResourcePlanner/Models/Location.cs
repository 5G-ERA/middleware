namespace Middleware.ResourcePlanner.Models;

internal record Location
{
    public string Name { get; init; }
    public string Type { get; init; }
    public string NetworkSlice { get; set; }
}