namespace Middleware.RedisInterface.Contracts.Responses;

public class FullTaskResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public string Priority { get; init; }
    public bool IsDeterministic { get; init; }
    public IEnumerable<string> Tags { get; init; } = new List<string>();

    public IEnumerable<FullActionResponse> ActionSequence { get; init; }
}

public class FullActionResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; } = default!;

    public string Priority { get; init; } = default!;
    public int Order { get; init; }
    public int? MinimumNumCores { get; init; }
    public long? MinimumRam { get; init; }
    public bool SingleNetAppEntryPoint { get; init; }
    public IEnumerable<string> Tags { get; init; } = Enumerable.Empty<string>();

    public IEnumerable<FullInstanceResponse> Services { get; init; }
}

public class FullInstanceResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public string Type { get; init; }
    public bool? IsReusable { get; init; }
    public IEnumerable<RosTopicResponse> RosTopicPublishers { get; init; } = Enumerable.Empty<RosTopicResponse>();
    public IEnumerable<RosTopicResponse> RosTopicSubscribers { get; init; } = Enumerable.Empty<RosTopicResponse>();
    public IEnumerable<RosActionResponse> RosActions { get; init; } = Enumerable.Empty<RosActionResponse>();
    public IEnumerable<RosServiceResponse> RosServices { get; init; } = Enumerable.Empty<RosServiceResponse>();
    public IEnumerable<RosTransformsResponse> RosTransforms { get; init; } = Enumerable.Empty<RosTransformsResponse>();
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
    public FullContainerResponse ContainerImage { get; set; }
}

public class FullContainerResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public DateTime LastUpdateTime { get; init; }
    public string Description { get; init; } = default!;

    // ReSharper disable once InconsistentNaming
    public string K8sDeployment { get; init; }

    // ReSharper disable once InconsistentNaming
    public string K8sService { get; init; } = default!;
}