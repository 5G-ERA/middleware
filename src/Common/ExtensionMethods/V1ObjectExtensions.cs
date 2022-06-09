﻿using k8s.Models;
using Microsoft.Extensions.Logging;

namespace Middleware.Common.ExtensionMethods;

public static class V1ObjectExtensions
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

    public static string GetExternalAddress(this V1Service service, ILogger logger = null)
    {
        var ingress = service.Status?.LoadBalancer?.Ingress?.FirstOrDefault();

        if (ingress is null)
        {
            return string.Empty;
        }

        logger?.LogInformation("Available ExternalIP: {externalIp}, ExternalName: {externalName}, IngressIP: {ingressIP}, " +
                              "IngressName: {ingressName}",
            service.Spec.ExternalIPs.FirstOrDefault(), service.Spec.ExternalName, ingress.Ip, ingress.Hostname);

        return ingress.Hostname ?? ingress.Ip;
    }

    public static string GetServiceLabelSelector(Guid serviceId)
    {
        return $"{ServiceIdSelector}={serviceId}";
    }
}