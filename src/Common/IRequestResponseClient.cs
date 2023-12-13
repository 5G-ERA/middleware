namespace Middleware.Common;

public interface IRequestResponseClient<in TRequest, TResponse> where TRequest : class where TResponse : class
{
    Task<TResponse> Request(TRequest request);
}