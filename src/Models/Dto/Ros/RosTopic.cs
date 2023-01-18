namespace Middleware.Models.Dto.Ros;

internal class RosTopic
{
    public string? Name { get; set; }
    public string? Type { get; set; }
    public string? Description { get; set; }
    public bool Enabled { get; set; }
}