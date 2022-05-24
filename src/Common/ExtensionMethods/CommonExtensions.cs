﻿using Amazon;
using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Middleware.Common.Config;

namespace Middleware.Common.ExtensionMethods;

public static class CommonExtensions
{
    public static IServiceCollection RegisterCommonServices(this IServiceCollection services)
    {
        services.AddSingleton<IEnvironment, MiddlewareEnvironment>();
        return services;
    }
    /// <summary>
    /// Register the AWS Secrets Manager to retrieve the data for the appsettings.json file
    /// </summary>
    /// <param name="builder"></param>
    /// <returns></returns>
    public static WebApplicationBuilder RegisterSecretsManager(this WebApplicationBuilder builder)
    {
        builder.Configuration.AddSecretsManager(region: RegionEndpoint.EUWest1, configurator: opt =>
        {
            opt.SecretFilter = entry => entry.Name.StartsWith($"{AppConfig.SystemName}-");
            opt.KeyGenerator = (_, s) => s
                .Replace($"{AppConfig.SystemName}-", string.Empty)
                .Replace("__", ":");
        });
        builder.Services.Configure<ElasticConfig>(builder.Configuration.GetSection(ElasticConfig.ConfigName));
        return builder;
    }
}