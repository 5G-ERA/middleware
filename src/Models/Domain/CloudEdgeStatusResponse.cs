namespace Middleware.Models.Domain;

public class CloudEdgeStatusResponse
{
    public Guid Id { get; set; }
    public string Type { get; set; }
    public bool IsOnline { get; set; }
    public DateTime LastUpdatedTime { get; set; }
}

public class CloudEdgeStatusRequest
{
    public string Type { get; set; }
    public bool IsOnline { get; set; }
}