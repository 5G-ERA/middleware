using Middleware.Common.Attributes;

namespace Middleware.Common.Enums;

public enum K8SServiceKind
{
    [StringValue("ClusterIP")]
    ClusterIp,

    [StringValue("NodePort")]
    NodePort,

    [StringValue("LoadBalancer")]
    LoadBalancer,

    [StringValue("ExternalName")]
    ExternalName
}