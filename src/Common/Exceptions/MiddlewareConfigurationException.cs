namespace Middleware.Common.Exceptions;

public class MiddlewareConfigurationException : Exception
{
    public string Reason { get; }

    public MiddlewareConfigurationException(string reason) : base(reason)
    {
        Reason = reason;
    }
}