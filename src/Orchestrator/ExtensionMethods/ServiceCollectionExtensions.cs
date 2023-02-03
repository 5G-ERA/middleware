using MassTransit;
using Middleware.Common.Config;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.Handlers;

namespace Middleware.Orchestrator.ExtensionMethods;

public static class ServiceCollectionExtensions
{
    public static IServiceCollection RegisterRabbitMqConsumers(this IServiceCollection services,
        RabbitMqConfig mqConfig)
    {
        // MassTransit-RabbitMQ Configuration
        services.AddOptions<MassTransitHostOptions>()
            .Configure(options =>
            {
                // if specified, waits until the bus is started before
                // returning from IHostedService.StartAsync
                // default is false
                options.WaitUntilStarted = true;

                // if specified, limits the wait time when starting the bus
                options.StartTimeout = TimeSpan.FromSeconds(10);

                // if specified, limits the wait time when stopping the bus
                options.StopTimeout = TimeSpan.FromSeconds(30);
            });
        services.AddMassTransit(config =>
        {
            config.SetKebabCaseEndpointNameFormatter();
            config.AddConsumer<DeployPlanConsumer, DeployPlanConsumerDefinition>();
            
            config.UsingRabbitMq((ctx, cfg) =>
            {
                cfg.ExchangeType = "direct";
                cfg.Durable = true;
                cfg.Host(mqConfig.Address, "/", h =>
                {
                    h.Username(mqConfig.User);
                    h.Password(mqConfig.Pass);
                });
                
                cfg.ConfigureEndpoints(ctx);
                
            });
        });
        return services;
    }
}