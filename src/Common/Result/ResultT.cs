namespace Middleware.Common.Result;

public readonly struct Result<T> : IResult<T>
{
    /// <inheritdoc />
    public bool IsFailure => !IsSuccess;
    public bool IsSuccess { get; }
    
    public string Error { get; }

    public T Value { get; }

    private Result(T value, string error = null!, bool isSuccess = true)
    {
        Value = value;
        Error = error;
        IsSuccess = isSuccess;
    }

    private Result(string error) : this(default!, error, false)
    {
    }
    
    public static Result<T> operator + (Result<T> left, Result right)
    {
        if (left.IsSuccess && right.IsSuccess)
        {
            return new Result<T>(left.Value);
        }

        if (left.IsSuccess)
        {
            return new Result<T>(right.Error);
        }

        if (right.IsSuccess)
        {
            return new Result<T>(left.Error);
        }

        return new Result<T>($"{left.Error}, {right.Error}");
    }
    public static Result<T> operator + (Result left, Result<T> right)
    {
        if (left.IsSuccess && right.IsSuccess)
        {
            return new Result<T>(right.Value);
        }

        if (left.IsSuccess)
        {
            return new Result<T>(right.Error);
        }

        if (right.IsSuccess)
        {
            return new Result<T>(left.Error);
        }

        return new Result<T>($"{left.Error}, {right.Error}");
    }
    
    public static Result<T> operator +(Result<T> left, Result<T> right)
    {
        if (left.IsSuccess && right.IsSuccess)
        {
            return new Result<T>(left.Value);
        }

        if (left.IsSuccess)
        {
            return new Result<T>(right.Error);
        }

        if (right.IsSuccess)
        {
            return new Result<T>(left.Error);
        }

        return new Result<T>($"{left.Error}, {right.Error}");
    }
    public static implicit operator Result<T>(T left)
    {
        return new Result<T>(left);
    }

    public static implicit operator Result<T>(string error)
    {
        return new Result<T>(error);
    }
    public static implicit operator Result<T>(Result result)
    {
        if (result.IsFailure)
        {
            return new Result<T>(result.Error);    
        }

        return new Result<T>(default!, null!, true);
    }

    public static Result<T> Combine(params Result<T>[] results)
    {
        if (results.Length == 0)
            throw new ArgumentException("Argument list cannot be empty", nameof(results));
        
        var isError = results.Any(r => r.IsSuccess == false);
        
        return isError
            ? new Result<T>(results.Select(r => r.Error).Aggregate((a, b) => $"{a}, {b}"))
            : new Result<T>(results.Select(r => r.Value).First());
    }
    public static Result<T> Success(T value) => new Result<T>(value);
    
}