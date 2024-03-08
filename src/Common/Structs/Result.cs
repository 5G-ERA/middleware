namespace Middleware.Common.Structs;

public struct Result<TValue>
{
    public bool IsSuccess { get; }
    public string ErrorMessage => Exception?.Message ?? string.Empty;
    public Exception? Exception { get; }

    public TValue Value { get; }

    private Result(TValue value, Exception? ex = default, bool isSuccess = true)
    {
        Value = value;
        Exception = ex;
        IsSuccess = isSuccess;
    }

    private Result(Exception? ex) : this(default!, ex, false)
    {
    }

    public static implicit operator Result<TValue>(TValue left)
    {
        return new Result<TValue>(left);
    }

    public static implicit operator Result<TValue>(Exception? right)
    {
        return new Result<TValue>(right);
    }
}