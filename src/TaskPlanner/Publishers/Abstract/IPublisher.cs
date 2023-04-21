using Middleware.Common.MessageContracts;

namespace Middleware.TaskPlanner.Publishers;

public interface IPublisher<in T> where T : Message
{
    Task PublishAsync(T message);
}