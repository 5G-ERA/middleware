using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Ros;

public class RosNode
{
    [Indexed]
    public string? Name { get; set; }
    [Indexed(JsonPath = "$.Name")]
    [Searchable(JsonPath = "$.Description")]
    public List<RosTopic> Publications { get; set; } = new();
    [Indexed(JsonPath = "$.Name")]
    [Searchable(JsonPath = "$.Description")]
    public List<RosTopic> Subscriptions { get; set; } = new();
    [Indexed(JsonPath = "$.Name")]
    [Searchable(JsonPath = "$.Description")]
    public List<RosService> Services { get; set; } = new();
}