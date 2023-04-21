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

    public static string TrimSuffix(this string s, string suffix)
    {
        if (s.EndsWith(suffix))
        {
            return s.Substring(0, s.Length - suffix.Length);
        }

        return s;
    }
}