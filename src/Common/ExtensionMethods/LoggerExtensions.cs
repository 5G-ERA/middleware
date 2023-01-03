using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Middleware.Common.Config;
using Serilog;
using Serilog.Sinks.Elasticsearch;

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
        var config = builder.Configuration.GetSection(ElasticConfig.ConfigName).Get<ElasticConfig>();
        return builder.UseElasticSerilogLogger(config);
    }
    /// <summary>
    /// Configures the Logging across the application to use Serilog and output the logs to the Elasticsearch
    /// </summary>
    /// <param name="builder"></param>
    /// <param name="elasticConfig">Configuration with the description of the connection to the Elasticsearch</param>
    /// <returns></returns>
    public static WebApplicationBuilder UseElasticSerilogLogger(this WebApplicationBuilder builder, ElasticConfig elasticConfig)
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

            if (Uri.IsWellFormedUriString(elasticConfig.Url, UriKind.RelativeOrAbsolute))
            {
                cfg.WriteTo.Elasticsearch(new ElasticsearchSinkOptions(new Uri(elasticConfig.Url))
                {
                    ModifyConnectionSettings = conn =>
                        conn.BasicAuthentication(elasticConfig.User, elasticConfig.Password),
                    ConnectionTimeout = new TimeSpan(0, 1, 0),
                    IndexFormat =
                        $"{builder.Configuration["ApplicationName"]}-logs-{builder.Environment.EnvironmentName.ToLower().Replace('.', '-')}-{DateTime.UtcNow:yyyy.MM}",
                    AutoRegisterTemplate = true,
                    EmitEventFailure = EmitEventFailureHandling.WriteToSelfLog |
                                       EmitEventFailureHandling.WriteToFailureSink |
                                       EmitEventFailureHandling.RaiseCallback,
                    FailureSink = new LoggerConfiguration().WriteTo.Console().CreateLogger()
                });
            }
        });
        builder.Services.AddSingleton<Serilog.ILogger>(Log.Logger);
        return builder;
    }
}
