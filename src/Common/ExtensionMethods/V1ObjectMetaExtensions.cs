using k8s.Models;

namespace Middleware.Common.ExtensionMethods;

public static class V1ObjectMetaExtensions
{
    private const string ServiceIdSelector = "serviceId";

    /// <summary>
    /// Sets the label for the object metadata with the specified serviceId
    /// </summary>
    /// <param name="meta"></param>
    /// <param name="serviceId"></param>
    public static void SetServiceLabel(this V1ObjectMeta meta, Guid serviceId)
    {
        if (meta.Labels is null)
        {
            meta.Labels = new Dictionary<string, string>();
        }
        meta.Labels.Add(ServiceIdSelector, serviceId.ToString());
    }

    public static string GetServiceLabelSelector(Guid serviceId)
    {
        return $"{ServiceIdSelector}={serviceId}";
    }
}