namespace Middleware.Common.ExtensionMethods;

public static class StringExtensions
{
    /// <summary>
    ///     Removes the not allowed characters so the name matches the correct k8s name format
    /// </summary>
    /// <param name="s"></param>
    /// <returns></returns>
    // ReSharper disable once InconsistentNaming
    public static string SanitizeAsK8sObjectName(this string s)
    {
        return s.Replace(" ", "-")
            .Replace('_', '-')
            .Replace(':', '-')
            .Replace('.', '-')
            .Replace('/', '-')
            .ToLower().Trim();
    }

    /// <summary>
    ///     Sanitizes the yaml string saved in Redis to the proper yaml format
    /// </summary>
    /// <param name="str"></param>
    /// <returns></returns>
    public static string SanitizeAsK8SYaml(this string str)
    {
        return str.Replace("\\n", "\n");
    }

    public static string TrimSuffix(this string s, string suffix)
    {
        return !s.EndsWith(suffix) ? s : s.Substring(0, s.Length - suffix.Length);
    }
}