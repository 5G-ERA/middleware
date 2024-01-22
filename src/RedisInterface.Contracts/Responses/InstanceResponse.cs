namespace Middleware.RedisInterface.Contracts.Responses;

public class InstanceResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public string Type { get; init; }
    public bool? IsReusable { get; init; }
    public IEnumerable<RosTopicResponse> RosTopicPublishers { get; init; } = Enumerable.Empty<RosTopicResponse>();
    public IEnumerable<RosTopicResponse> RosTopicSubscribers { get; init; } = Enumerable.Empty<RosTopicResponse>();
    public IEnumerable<RosActionResponse> RosActions { get; init; } = Enumerable.Empty<RosActionResponse>();
    public IEnumerable<RosServiceResponse> RosService { get; init; } = Enumerable.Empty<RosServiceResponse>();
    public int RosVersion { get; init; }
    public string RosDistro { get; init; } = default!;
    public string Family { get; init; }

    [Obsolete("\"MinimumRam\" is not longer used, please use \"Ram\" compound property")]
    public long? MinimumRam { get; init; }

    [Obsolete("\"MinimumNumOfCores\" is not longer used, please use \"NumberOfCores\" compound property")]
    public int? MinimumNumOfCores { get; init; }

    public RequirementResponse Ram { get; init; }
    public RequirementResponse NumberOfCores { get; init; }
    public RequirementResponse DiskStorage { get; init; }
    public RequirementResponse Throughput { get; init; }
    public RequirementResponse Latency { get; init; }

    public DateTime OnboardedTime { get; init; }
    public IEnumerable<string> Tags { get; init; } = Enumerable.Empty<string>();
    public IEnumerable<string> AppliedPolicies { get; init; } = Enumerable.Empty<string>();
}