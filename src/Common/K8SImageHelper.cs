namespace Middleware.Common;

public class K8SImageHelper
{
    /// <summary>
    /// Combines the information about the image registry, repository and tag to form the full image name
    /// </summary>
    /// <param name="registry"></param>
    /// <param name="repositoryName"></param>
    /// <param name="tag"></param>
    /// <returns></returns>
    public static string BuildImageName(string registry, string repositoryName, string tag)
    {
        return $"{registry}/{repositoryName}:{tag}";
    }
}