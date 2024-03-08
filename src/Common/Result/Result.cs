namespace Middleware.Common.Result;

public class Result : IResult
{
    /// <inheritdoc />
    public bool IsFailure => !IsSuccess;

    /// <inheritdoc />
    public bool IsSuccess { get; }
    
    public string Error { get; }

    public Result()
    {
        IsSuccess = true;
        Error = null!;
    }
    private Result(string error)
    {
        Error = error;
        IsSuccess = false;
    }
    public static Result operator +(Result left, Result right)
    {
        if (left.IsSuccess && right.IsSuccess)
        {
            return new Result();
        }

        if (left.IsSuccess)
        {
            return new Result(right.Error);
        }

        if (right.IsSuccess)
        {
            return new Result(left.Error);
        }

        return new Result($"{left.Error}, {right.Error}");
    }

    public static implicit operator Result(string error)
    {
        return new Result(error);
    }
    public static Result Success() => new Result();
    public static Result Failure(string error) => new Result(error);
}