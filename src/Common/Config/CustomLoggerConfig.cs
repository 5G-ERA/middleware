namespace Middleware.Common.Config;

public class CustomLoggerConfig
{
    /// <summary>
    /// Name of the section in the appsettings.json file
    /// </summary>
    public const string ConfigName = "CustomLogger";
    /// <summary>
    /// Name of the Logger that will be used. Currently available are Loki or Elasticsearch
    /// </summary>
    public string LoggerName { get; set; }
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