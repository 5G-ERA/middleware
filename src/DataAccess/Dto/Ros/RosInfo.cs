using Redis.OM.Modeling;

namespace Middleware.DataAccess.Dto.Ros;

internal class RosInfo
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
