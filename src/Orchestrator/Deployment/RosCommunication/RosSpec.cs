using Middleware.Models.Domain.Ros;

namespace Middleware.Orchestrator.Deployment.RosCommunication;

public class RosSpec
{
    public IReadOnlyList<RosTopicModel> TopicSubscribers { get; init; }
    public IReadOnlyList<RosTopicModel> TopicPublishers { get; init; }
    public IReadOnlyList<RosServiceModel> ServiceSubscribers { get; init; }
    public IReadOnlyList<RosTransformsModel> TransformsPublishers { get; init; }
    public IReadOnlyList<RosActionModel> ActionSubscribers { get; init; }
    
    public RosSpec(IReadOnlyList<RosTopicModel> topicSubscribers,
        IReadOnlyList<RosTopicModel> topicPublishers,
        IReadOnlyList<RosServiceModel> serviceSubscribers,
        IReadOnlyList<RosTransformsModel> transformsPublishers,
        IReadOnlyList<RosActionModel> actionSubscribers)
    {
        TopicSubscribers = topicSubscribers ?? throw new ArgumentNullException(nameof(topicSubscribers));
        TopicPublishers = topicPublishers ?? throw new ArgumentNullException(nameof(topicPublishers));
        ServiceSubscribers = serviceSubscribers;
        TransformsPublishers = transformsPublishers;
        ActionSubscribers = actionSubscribers;
    }
}