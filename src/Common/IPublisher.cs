using Middleware.Common.MessageContracts;

namespace Middleware.Common;

public interface IPublisher<in T> where T : Message
{
    Task PublishAsync(T message);
}