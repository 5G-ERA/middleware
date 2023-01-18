namespace Middleware.Models.Dto.Ros;

internal class Sensor
{
    public string? Name { get; set; }
    public string? Type { get; set; }
    public string? Description { get; set; }
    public List<string> Nodes { get; set; } = new();
}