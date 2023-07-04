using ValueOf;

namespace Middleware.Models.Domain.ValueObjects;

public class TopicName : ValueOf<string, TopicName>
{
    /// <inheritdoc />
    protected override void Validate()
    {
        if (!Value.StartsWith('/'))
            throw new ArgumentException("TopicName must start with '/'");
    }

    public override string ToString()
    {
        return Value;
    }
}

public class TopicNameConverter : ValueObjectConverter<TopicName>
{
    /// <inheritdoc />
    protected override string ConvertValueToString(TopicName value)
    {
        return value.Value;
    }
}