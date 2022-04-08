namespace Middleware.Common.Attributes;

public class StringValueAttribute : Attribute
{
    /// <summary>
    /// Holds the StringValue attribute value in enum
    /// </summary>
    public string StringValue { get; protected set; }

    /// <summary>
    /// Initializes the StringValue
    /// </summary>
    public StringValueAttribute(string value)
    {
        StringValue = value;
    }

}