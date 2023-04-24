namespace Middleware.CentralApi.Contracts.Responses;

public class LocationResponse
{
    public Guid Id { get; init; }
    public string Organization { get; init; } = default!;
    public string Name { get; init; } = default!;
    public string Type { get; init; } = default!;
    public bool IsOnline { get; init; } = true;
    
    public Uri? Address { get; init; }
}