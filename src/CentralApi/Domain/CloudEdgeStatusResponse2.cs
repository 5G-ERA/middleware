namespace Middleware.CentralApi.Domain;


public class CloudEdgeStatusResponse2
{
    public Guid Id { get; set; }
    public string Type { get; set; }
    public bool IsOnline { get; set; }
    public DateTime LastUpdatedTime { get; set; }
}
public class CloudEdgeStatusRequest2
{
    public string Type { get; set; }
    public bool IsOnline { get; set; }
}