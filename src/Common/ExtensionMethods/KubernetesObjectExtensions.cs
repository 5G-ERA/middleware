using k8s.Models;
using Microsoft.Extensions.Logging;

namespace Middleware.Common.ExtensionMethods;

public static class KubernetesObjectExtensions
{
    private const string NetAppIdSelector = "serviceId";
    private const int WebsocketPort = 80;

    /// <summary>
    ///     Sets the label for the object metadata with the specified serviceId
    /// </summary>
    /// <param name="meta"></param>
    /// <param name="serviceId"></param>
    public static void SetServiceLabel(this V1ObjectMeta meta, Guid serviceId)
    {
        if (meta.Labels is null) meta.Labels = new Dictionary<string, string>();
        meta.Labels.Add(NetAppIdSelector, serviceId.ToString());
    }

    [Obsolete(
        "The Middleware does not use Multus for multiple Network Attachments to pods. Setting this annotation will not have any effect.")]
    public static void AddNetAppMultusAnnotations(this V1ObjectMeta meta, string networkName)
    {
        const string annotationKey = "k8s.v1.cni.cncf.io/networks";
        if (meta.Annotations is null) meta.Annotations = new Dictionary<string, string>();

        meta.Annotations[annotationKey] = networkName;
    }

    public static string GetExternalAddress(this V1Service service, ILogger logger = null)
    {
        var ingress = service.Status?.LoadBalancer?.Ingress?.FirstOrDefault();

        if (ingress is null) return string.Empty;

        logger?.LogInformation(
            "Available ExternalIP: {externalIp}, ExternalName: {externalName}, IngressIP: {ingressIP}, " +
            "IngressName: {ingressName}",
            service.Spec.ExternalIPs?.FirstOrDefault(), service.Spec?.ExternalName, ingress.Ip, ingress.Hostname);

        return ingress.Hostname ?? ingress.Ip;
    }

    /// <summary>
    ///     Returns the selector definition for the NetApp
    /// </summary>
    /// <param name="serviceId"></param>
    /// <returns></returns>
    public static string GetNetAppLabelSelector(Guid serviceId)
    {
        return $"{NetAppIdSelector}={serviceId}";
    }

    public static IReadOnlyList<string> GetDeploymentNames(this V1DeploymentList deployments)
    {
        return deployments.Items.Select(d => d.Metadata.Name).OrderBy(d => d).ToList();
    }

    public static bool ContainsWebsocketCompatiblePort(this V1Service service)
    {
        var ports = service.Spec.Ports.Select(p => p.Port).ToList();

        return ports.Contains(WebsocketPort);
    }

    public static void AddWebsocketCompatiblePort(this V1Service service)
    {
        service.Spec.Ports.Add(new(WebsocketPort, name: "websocket"));
    }
}