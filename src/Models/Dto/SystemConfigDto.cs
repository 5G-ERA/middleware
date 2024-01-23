﻿using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "system-cfg-idx", StorageType = StorageType.Json, Prefixes = new[] { Prefix })]
public class SystemConfigDto : Dto
{
    public const string Prefix = "Config";

    /// <inheritdoc />
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;

    [Indexed]
    public string Ros2RelayContainer { get; init; } = default!;

    [Indexed]
    public string Ros1RelayContainer { get; init; } = default!;

    [Indexed]
    public string RosInterRelayNetAppContainer { get; init; } = default!;

    [Indexed]
    public int HeartbeatExpirationInMinutes { get; set; }

    /// <inheritdoc />
    public override BaseModel ToModel()
    {
        var d = this;
        return new SystemConfigModel
        {
            Id = Guid.Empty,
            Name = "System Config",
            Ros1RelayContainer = d.Ros1RelayContainer,
            Ros2RelayContainer = d.Ros2RelayContainer,
            RosInterRelayNetAppContainer = d.RosInterRelayNetAppContainer,
            HeartbeatExpirationInMinutes = d.HeartbeatExpirationInMinutes
        };
    }
}