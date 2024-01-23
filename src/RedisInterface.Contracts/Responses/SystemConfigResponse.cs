namespace Middleware.RedisInterface.Contracts.Responses;

public class SystemConfigResponse
{
    public string Ros2RelayContainer { get; set; } = default!;
    public string Ros1RelayContainer { get; set; } = default!;
    public string RosInterRelayNetAppContainer { get; set; } = default!;
    public int HeartbeatExpirationInMinutes { get; set; }
}