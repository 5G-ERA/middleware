using Middleware.Models.Domain;

namespace Middleware.RedisInterface.Contracts.Responses;

public class InstanceResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public string Type { get; init; }
    public bool? IsReusable { get; init; }
    public IEnumerable<RosTopicModel> RosTopicPublishers { get; init; } = Enumerable.Empty<RosTopicModel>();
    public IEnumerable<RosTopicModel> RosTopicSubscribers { get; init; } = Enumerable.Empty<RosTopicModel>();
    public int RosVersion { get; init; }
    public string RosDistro { get; init; } = default!;
    public string Family { get; init; }
    public long? MinimumRam { get; init; }
    public int? MinimumNumOfCores { get; init; }
    public DateTime OnboardedTime { get; init; }
    public IEnumerable<string>? Tags { get; init; } = Enumerable.Empty<string>();
    public IEnumerable<string> AppliedPolicies { get; init; } = Enumerable.Empty<string>();
}