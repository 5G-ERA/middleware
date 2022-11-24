namespace Middleware.Common.ExtensionMethods;

public static class TypeExtensions
{
    /// <summary>
    /// Returns class name without trailing `Model` in the name
    /// </summary>
    /// <param name="type"></param>
    /// <returns></returns>
    public static string GetModelName(this Type type)
    {
        return type.Name.EndsWith("Model")
            ? type.Name.Remove(type.Name.LastIndexOf("Model", StringComparison.Ordinal))
            : type.Name;
    }
}