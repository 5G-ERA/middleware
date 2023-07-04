namespace Middleware.Common;

public interface IPublisher<in T>
{
    Task PublishAsync(T message);
}