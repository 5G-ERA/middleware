namespace Middleware.Common.ExtensionMethods;

public static class StringExtensions
{
    /// <summary>
    /// Sanitizes the yaml string saved in Redis to the proper yaml format
    /// </summary>
    /// <param name="str"></param>
    /// <returns></returns>
    public static string SanitizeAsK8SYaml(this string str)
    {
        return str.Replace("\\n", "\n");
    }
}
