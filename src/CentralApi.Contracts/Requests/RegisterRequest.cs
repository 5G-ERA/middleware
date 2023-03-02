namespace Middleware.CentralApi.Contracts.Requests;

public class RegisterRequest
{
    public string Organization { get; init; }
    public string Name { get; init; }
    public string Type { get; init; }
}