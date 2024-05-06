namespace Middleware.RedisInterface.Contracts.Requests;

public class SystemConfigRequest
{
    public string Ros2RelayContainer { get; set; } = default!;
    public string Ros1RelayContainer { get; set; } = default!;
    public string RosInterRelayNetAppContainer { get; set; } = default!;
    public int HeartbeatExpirationInMinutes { get; set; }
    public string HermesContainer { get; set; } = default!;
    public string S3DataPersistenceRegion { get; set; } = default!;
    public string S3DataPersistenceBucketName { get; set; } = default!;
}