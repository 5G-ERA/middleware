using System.Reflection;
using Middleware.Common.Attributes;

namespace Middleware.Common.ExtensionMethods;

public static class EnumExtensions
{
    public static string GetStringValue(this Enum value)
    {
        Type type = value.GetType();

        FieldInfo fieldInfo = type.GetField(value.ToString());

        StringValueAttribute[] attrs = fieldInfo.GetCustomAttributes(typeof(StringValueAttribute), false) as StringValueAttribute[];

        return attrs != null && attrs.Length > 0 ? attrs[0].StringValue : null;
    }
}