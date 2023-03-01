using Microsoft.AspNetCore.Mvc;

namespace Middleware.RedisInterface.Contracts.Requests;

public class UpdateRobotTopicRequest
{
    [FromRoute]
    public Guid Id { get; set; }

    [FromQuery]
    public string TopicName { get; set; }
    
    [FromQuery]
    public bool Enabled { get; set; }
}