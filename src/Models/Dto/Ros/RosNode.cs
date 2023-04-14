using Middleware.Models.Domain;
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

    public ROSNodeModel ToModel()
    {
        return new ROSNodeModel()
        {
            Name = this.Name!,
            Publications = this.Publications.Select(x => x.ToModel()).ToList(),
            Services = this.Services.Select(x => x.ToModel()).ToList(),
            Subscriptions = this.Subscriptions.Select(x => x.ToModel()).ToList()
        };
    }
}