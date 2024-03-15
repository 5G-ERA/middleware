namespace Middleware.RedisInterface.Contracts.Requests;

public class InstanceRequest
{
    public string Name { get; init; }
    public string Type { get; init; }
    public bool IsReusable { get; init; }
    public bool IsPersistent { get; init; }
    public IEnumerable<RosTopicRequest> RosTopicPublishers { get; init; } = Enumerable.Empty<RosTopicRequest>();
    public IEnumerable<RosTopicRequest> RosTopicSubscribers { get; init; } = Enumerable.Empty<RosTopicRequest>();
    public IEnumerable<RosActionRequest> RosActions { get; init; } = Enumerable.Empty<RosActionRequest>();
    public IEnumerable<RosServiceRequest> RosServices { get; init; } = Enumerable.Empty<RosServiceRequest>();
    public IEnumerable<RosTransformsRequest> RosTransforms { get; init; } = Enumerable.Empty<RosTransformsRequest>();
    public int RosVersion { get; init; }
    public string RosDistro { get; init; } = default!;
    public string Family { get; init; }

    [Obsolete("\"MinimumRam\" is not longer used, please use \"Ram\" compound property")]
    public int? MinimumRam { get; init; }

    [Obsolete("\"MinimumNumOfCores\" is not longer used, please use \"NumberOfCores\" compound property")]
    public int? MinimumNumOfCores { get; init; }

    public RequirementRequest Ram { get; init; }
    public RequirementRequest NumberOfCores { get; init; }
    public RequirementRequest DiskStorage { get; init; }
    public RequirementRequest Throughput { get; init; }
    public RequirementRequest Latency { get; init; }


    public IEnumerable<string> Tags { get; init; } = Enumerable.Empty<string>();
    public IEnumerable<string> AppliedPolicies { get; init; } = Enumerable.Empty<string>();
}