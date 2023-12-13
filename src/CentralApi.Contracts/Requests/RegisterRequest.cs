namespace Middleware.CentralApi.Contracts.Requests;

public class RegisterRequest
{
    public string Organization { get; init; } = default!;
    public string Name { get; init; }= default!;
    public string Type { get; init; }= default!;
    public string Address { get; init; }= default!;
}