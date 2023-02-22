namespace Middleware.Common.Config;

public class MiddlewareConfig
{
    public const string ConfigName = "Middleware";

    public string Organization { get; init; }
    public string InstanceName { get; init; }
    public string InstanceType { get; init; }
}