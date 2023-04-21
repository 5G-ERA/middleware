namespace Middleware.Common.Structs;

public struct Either<TLeft, TRight>
{
    private readonly TLeft _left;
    private readonly TRight _right;
    private readonly bool _isLeft;

    private Either(TLeft left, TRight right, bool isLeft)
    {
        _left = left;
        _right = right;
        _isLeft = isLeft;
    }

    private Either(TLeft left) : this(left, default, true)
    {
    }

    private Either(TRight right) : this(default, right, false)
    {
    }

    public static implicit operator Either<TLeft, TRight>(TLeft left)
    {
        return new Either<TLeft, TRight>(left);
    }

    public static implicit operator Either<TLeft, TRight>(TRight right)
    {
        return new Either<TLeft, TRight>(right);
    }

    public T Match<T>(Func<TLeft, T> left, Func<TRight, T> right)
    {
        return _isLeft ? left(_left) : right(_right);
    }

    public void Act(Action<TLeft> left, Action<TRight> right)
    {
        if (_isLeft)
        {
            left.Invoke(_left);
        }
        else
        {
            right.Invoke(_right);
        }
        
    }
}

