namespace Middleware.Common.Config;
public class GatewayConfig
{
    public const string ConfigName = "GatewayConfig";

    public int NodePort { get; init; }
    public bool IsIngress { get; init; }
}
