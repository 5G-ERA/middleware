using Middleware.Models.Domain.Ros;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Ros;

public class RosNode
{
    [Indexed]
    public string? Name { get; init; }

    [Indexed(JsonPath = "$.Name")]
    [Searchable(JsonPath = "$.Description")]
    public List<RosTopic> Publications { get; init; } = new();

    [Indexed(JsonPath = "$.Name")]
    [Searchable(JsonPath = "$.Description")]
    public List<RosTopic> Subscriptions { get; init; } = new();

    [Indexed(JsonPath = "$.Name")]
    [Searchable(JsonPath = "$.Description")]
    public List<RosService> Services { get; init; } = new();

    public RosNodeModel ToModel()
    {
        return new()
        {
            Name = Name!,
            Publications = Publications.Select(x => x.ToModel()).ToList(),
            Services = Services.Select(x => x.ToModel()).ToList(),
            Subscriptions = Subscriptions.Select(x => x.ToModel()).ToList()
        };
    }
}