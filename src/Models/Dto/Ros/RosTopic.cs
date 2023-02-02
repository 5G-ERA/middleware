using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Ros;

public class RosTopic 
{
    public string? Name { get; set; }
    public string? Type { get; set; }
    public string? Description { get; set; }
    public bool Enabled { get; set; }

}