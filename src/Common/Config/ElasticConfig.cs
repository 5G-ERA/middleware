namespace Middleware.Common.Config;

public class ElasticConfig
{
    /// <summary>
    /// Name of teh section in the appsettings.json file
    /// </summary>
    public const string ConfigName = "Elasticsearch";
    /// <summary>
    /// Url of the service
    /// </summary>
    public string Url { get; set; }
    /// <summary>
    /// User name
    /// </summary>
    public string User { get; set; }
    /// <summary>
    /// Password
    /// </summary>
    public string Password { get; set; }
}