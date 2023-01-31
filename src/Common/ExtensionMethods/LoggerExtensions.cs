using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Middleware.Common.Config;
using Serilog;
using Serilog.Sinks.Elasticsearch;
using Serilog.Sinks.Grafana.Loki;

//using Serilog.Sinks.Loki;

namespace Middleware.Common.ExtensionMethods;

public static class LoggerExtensions
{
    /// <summary>
    /// Configures the Logging across the application to use Serilog and output the logs to the Elasticsearch. 
    /// </summary>
    /// <param name="builder"></param>
    /// <returns></returns>
    public static WebApplicationBuilder UseElasticSerilogLogger(this WebApplicationBuilder builder)
    {
        var config = builder.Configuration.GetSection(CustomLoggerConfig.ConfigName).Get<CustomLoggerConfig>();
        return builder.UseElasticSerilogLogger(config);
    }

    public static WebApplicationBuilder ConfigureDefaultLogger(this WebApplicationBuilder builder)
    {
#if DEBUG
        Serilog.Debugging.SelfLog.Enable(msg => Console.WriteLine(msg));
#endif

        builder.Host.UseSerilog((ctx, cfg) =>
        {
            cfg.Enrich.FromLogContext()
                .Enrich.WithMachineName()
                .Enrich.WithEnvironmentUserName()
                .Enrich.WithProperty("Environment", builder.Environment.EnvironmentName).ReadFrom
                .Configuration(ctx.Configuration)
                .WriteTo.Console();
        });
        builder.Services.AddSingleton<Serilog.ILogger>(Log.Logger);
        return builder;
    }

    /// <summary>
    /// Configures the Logging across the application to use Serilog and output the logs to the Elasticsearch
    /// </summary>
    /// <param name="builder"></param>
    /// <param name="config">Configuration with the description of the connection to the log aggregation tool</param>
    /// <param name="appName">Name of the application that will be used when filtering the logs</param>
    /// <returns></returns>
    public static WebApplicationBuilder UseElasticSerilogLogger(this WebApplicationBuilder builder,
        CustomLoggerConfig config, string appName = "Middleware")
    {
        builder.ConfigureDefaultLogger();
        if (Uri.IsWellFormedUriString(config.Url, UriKind.RelativeOrAbsolute))
        {
            builder.Host.UseSerilog((ctx, cfg) =>
            {
                cfg.WriteTo.GrafanaLoki(config.Url, labels: new[] { new LokiLabel() { Key = "app", Value = appName } },
                    credentials: new LokiCredentials() { Login = config.User, Password = config.Password });
            });
            
            
            // cfg.WriteTo.Elasticsearch(new ElasticsearchSinkOptions(new Uri(config.Url))
            // {
            //     ModifyConnectionSettings = conn =>
            //         conn.BasicAuthentication(config.User, config.Password),
            //     ConnectionTimeout = new TimeSpan(0, 1, 0),
            //     IndexFormat =
            //         $"{builder.Configuration["ApplicationName"]}-logs-{builder.Environment.EnvironmentName.ToLower().Replace('.', '-')}-{DateTime.UtcNow:yyyy.MM}",
            //     AutoRegisterTemplate = true,
            //     EmitEventFailure = EmitEventFailureHandling.WriteToSelfLog |
            //                        EmitEventFailureHandling.WriteToFailureSink |
            //                        EmitEventFailureHandling.RaiseCallback,
            //     FailureSink = new LoggerConfiguration().WriteTo.Console().CreateLogger()
            // });
        }

        return builder;
    }
}