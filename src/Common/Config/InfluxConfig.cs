
namespace Middleware.Common.Config;
public class InfluxConfig
{
    public const string ConfigName = "Influx";

    public string ApiKey { get; init; }
    public string Address { get; init; }
}
