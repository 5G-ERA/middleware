using System.Text;
using IdentityModel;

namespace Middleware.Common;

public static class K8SImageHelper
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
        if (string.IsNullOrWhiteSpace(repositoryName))
            throw new ArgumentException("Repository name not provided.", nameof(repositoryName));

        StringBuilder builder =new StringBuilder();
        builder.Append(repositoryName);

        if (string.IsNullOrWhiteSpace(registry) == false)
        {
            builder.Insert(0, registry + "/");
        }

        if (string.IsNullOrWhiteSpace(tag) == false)
        {
            builder.Append($":{tag}");
        }
        return builder.ToString();
    }

    public static string GetTag(string image)
    {
        var splitted = image.Split(':');

        if (splitted.Length == 1)
            return "latest";

        return splitted[1];
    }
}