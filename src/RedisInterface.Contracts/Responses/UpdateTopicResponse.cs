namespace Middleware.RedisInterface.Contracts.Responses;

public class UpdateTopicResponse
{
    public Guid RobotId { get; set; }

    public string TopicName { get; set; } = default!;
    public string TopicType { get; set; } = default!;
    public string? TopicDescription { get; set; } = default!;
    public bool TopicEnabled { get; set; }
}