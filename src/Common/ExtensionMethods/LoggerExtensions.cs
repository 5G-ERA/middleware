using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Serilog;
using Serilog.Sinks.Elasticsearch;
using Serilog.Sinks.Grafana.Loki;

namespace Middleware.Common.ExtensionMethods;

public static class LoggerExtensions
{
    /// <summary>
    /// Configures the Logging across the application to use Serilog and output the logs to the configured log storage. 
    /// </summary>
    /// <param name="builder"></param>
    /// <returns></returns>
    public static WebApplicationBuilder ConfigureLogger(this WebApplicationBuilder builder)
    {
        var loggerConfig = builder.Configuration.GetSection(CustomLoggerConfig.ConfigName).Get<CustomLoggerConfig>();
#if DEBUG
        Serilog.Debugging.SelfLog.Enable(Console.WriteLine);
#endif
        var appName = builder.Configuration["ApplicationName"];
        builder.Host.UseSerilog((ctx, cfg) =>
        {
            cfg.Enrich.FromLogContext()
                .Enrich.WithMachineName()
                .Enrich.WithEnvironmentUserName()
                .Enrich.WithProperty("Environment", builder.Environment.EnvironmentName).ReadFrom
                .Configuration(ctx.Configuration)
                .WriteTo.Console();

            if (loggerConfig is null)
                return;
            
            if (loggerConfig.LoggerName == Loggers.Elasticsearch.ToString())
            {
                var indexFormat =
                    $"{builder.Configuration["ApplicationName"]}-logs-{builder.Environment.EnvironmentName.ToLower().Replace('.', '-')}-{DateTime.UtcNow:yyyy.MM}";
                ConfigureElasticsearch(cfg, loggerConfig, indexFormat);
            }
            else if (loggerConfig.LoggerName == Loggers.Loki.ToString())
            {
                ConfigureLoki(cfg, loggerConfig, appName);
            }
        });
        builder.Services.AddSingleton<Serilog.ILogger>(Log.Logger);
        return builder;
    }

    private static void ConfigureLoki(LoggerConfiguration cfg, CustomLoggerConfig loggerConfig, string appName)
    {
        if (Uri.IsWellFormedUriString(loggerConfig.Url, UriKind.Absolute) == false)
        {
            throw new ArgumentException("The provided URL for the logger configuration is in the incorrect format",
                nameof(loggerConfig.Url));
        }

        cfg.WriteTo.GrafanaLoki(loggerConfig.Url, labels: new[] { new LokiLabel() { Key = "app", Value = appName } },
            credentials: new LokiCredentials() { Login = loggerConfig.User, Password = loggerConfig.Password });
    }

    private static void ConfigureElasticsearch(LoggerConfiguration cfg, CustomLoggerConfig loggerConfig,
        string indexFormat)
    {
        if (Uri.IsWellFormedUriString(loggerConfig.Url, UriKind.Absolute) == false)
        {
            throw new ArgumentException("The provided URL for the logger configuration is in the incorrect format",
                nameof(loggerConfig.Url));
        }

        cfg.WriteTo.Elasticsearch(new ElasticsearchSinkOptions(new Uri(loggerConfig.Url))
        {
            ModifyConnectionSettings = conn =>
                conn.BasicAuthentication(loggerConfig.User, loggerConfig.Password),
            ConnectionTimeout = new TimeSpan(0, 1, 0),
            IndexFormat = indexFormat,
            AutoRegisterTemplate = true,
            EmitEventFailure = EmitEventFailureHandling.WriteToSelfLog |
                               EmitEventFailureHandling.WriteToFailureSink |
                               EmitEventFailureHandling.RaiseCallback,
            FailureSink = new LoggerConfiguration().WriteTo.Console().CreateLogger()
        });
    }
}