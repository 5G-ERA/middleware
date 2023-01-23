namespace Middleware.Models.Exceptions;

[Serializable]
public class MappingException : Exception
{
    public Type TypeFrom { get; set; }
    public Type TypeTo { get; set; }

    public MappingException(Type typeFrom, Type typeTo) : base(message: $"Cannot convert {typeFrom} to {typeTo}")
    {
        TypeFrom = typeFrom;
        TypeTo = typeTo;
    }
}