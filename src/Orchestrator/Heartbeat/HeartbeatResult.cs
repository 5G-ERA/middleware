namespace Middleware.Orchestrator.Heartbeat;

public class HeartbeatResult<T>
{
    public bool IsSuccess { get; }
    public string ErrMessage { get; }
    public bool NotFound { get; }
    public T Value { get; }

    public HeartbeatResult(string errMessage, bool notFound = false)
    {
        ErrMessage = errMessage;
        NotFound = notFound;
        IsSuccess = false;
    }

    public HeartbeatResult(T value)
    {
        Value = value;
        IsSuccess = true;
    }
}