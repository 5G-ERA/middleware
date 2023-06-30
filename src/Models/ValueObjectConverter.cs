using System.Text.Json;
using System.Text.Json.Serialization;

namespace Middleware.Models;

public class ValueObjectConverter<T> : JsonConverter<T>
{
    /// <inheritdoc />
    public override T? Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options)
    {
        throw new NotImplementedException();
    }

    /// <inheritdoc />
    public override void Write(Utf8JsonWriter writer, T value, JsonSerializerOptions options)
    {
        var stringValue = ConvertValueToString(value);
        writer.WriteStringValue(stringValue);
    }

    protected virtual string ConvertValueToString(T value)
    {
        // Implement the conversion logic to a simple value here
        throw new NotImplementedException();
    }
}