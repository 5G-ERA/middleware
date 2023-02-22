using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Ros;

public class RosInfo
{
    [Indexed]
    public int RosVersion { get; set; }
    [Indexed]
    public string? RosDistro { get; set; }
    [Indexed]
    public Uri? RosRepo { get; set; }
    
    [Indexed(JsonPath = "$.Name")]
    public List<RosNode> RosNodes { get; set; } = new();
}