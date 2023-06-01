﻿using Middleware.Models.Domain;

namespace Middleware.RedisInterface.Contracts.Requests;

public class InstanceRequest
{
    public string Name { get; init; }
    public string Type { get; init; }
    public bool IsReusable { get; init; }
    public IEnumerable<RosTopicModel> RosTopicPublishers { get; init; } = Enumerable.Empty<RosTopicModel>();
    public IEnumerable<RosTopicModel> RosTopicSubscribers { get; init; } = Enumerable.Empty<RosTopicModel>();
    public int RosVersion { get; init; }
    public string RosDistro { get; init; } = default!;
    public string Family { get; init; }
    public int? MinimumRam { get; init; }
    public int? MinimumNumOfCores { get; init; }
    public IEnumerable<string> Tags { get; init; } = Enumerable.Empty<string>();
    public IEnumerable<string> AppliedPolicies { get; init; } = Enumerable.Empty<string>();
}