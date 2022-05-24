using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.DependencyInjection;
using Serilog;
using Serilog.Sinks.Elasticsearch;

namespace Middleware.Common.ExtensionMethods;

public static class LoggerExtensions
{
    /// <summary>
    /// Configures the Logging across the application to use Serilog and output the logs to the Elasticsearch
    /// </summary>
    /// <param name="builder"></param>
    /// <returns></returns>
    public static WebApplicationBuilder UseElasticSerilogLogger(this WebApplicationBuilder builder)
    {
#if DEBUG
        Serilog.Debugging.SelfLog.Enable(msg => Console.WriteLine(msg));
#endif

        Uri elasticSearchUri = new Uri("https://elastic.uri");

        builder.Host.UseSerilog((ctx, cfg) =>
        {
            cfg.Enrich.FromLogContext()
                .Enrich.WithMachineName()
                .Enrich.WithEnvironmentUserName()
                .WriteTo.Console()
                .WriteTo.Elasticsearch(new ElasticsearchSinkOptions(elasticSearchUri)
                {
                    ModifyConnectionSettings = conn => conn.BasicAuthentication("user", "pass"),
                    ConnectionTimeout = new TimeSpan(0, 1, 0),
                    IndexFormat =
                        $"{builder.Configuration["ApplicationName"]}-logs-{builder.Environment.EnvironmentName.ToLower().Replace('.', '-')}-{DateTime.UtcNow:yyyy.MM}",
                    AutoRegisterTemplate = true,
                    EmitEventFailure = EmitEventFailureHandling.WriteToSelfLog |
                                       EmitEventFailureHandling.WriteToFailureSink |
                                       EmitEventFailureHandling.RaiseCallback,
                    FailureSink = new LoggerConfiguration().WriteTo.Console().CreateLogger()
                })
                .Enrich.WithProperty("Environment", builder.Environment.EnvironmentName).ReadFrom.Configuration(ctx.Configuration);
        });
        builder.Services.AddSingleton<Serilog.ILogger>(Log.Logger);
        return builder;
    }
}