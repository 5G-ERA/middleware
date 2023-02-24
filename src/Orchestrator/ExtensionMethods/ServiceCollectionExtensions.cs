using MassTransit;
using Middleware.Common.Config;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.Handlers;
using RabbitMQ.Client;
using static MassTransit.Logging.OperationName;

namespace Middleware.Orchestrator.ExtensionMethods;

public static class ServiceCollectionExtensions
{
    public static IServiceCollection RegisterRabbitMqConsumers(this IServiceCollection services,
        RabbitMqConfig mqConfig, MiddlewareConfig mwConfig)
    {
        services.AddMassTransit(x =>
        {
            services.AddScoped<DeployPlanConsumer>();
            x.AddConsumer<DeployPlanConsumer>();
            x.UsingRabbitMq((busRegistrationContext, mqBusFactoryConfigurator) =>
            {
                //mqBusFactoryConfigurator.SetKebabCaseEndpointNameFormatter();
                mqBusFactoryConfigurator.ExchangeType = "direct";
                mqBusFactoryConfigurator.Durable = true;
                mqBusFactoryConfigurator.Host(mqConfig.Address, "/", hostConfig =>
                {
                    hostConfig.Username(mqConfig.User);
                    hostConfig.Password(mqConfig.Pass);
                });

                mqBusFactoryConfigurator.ReceiveEndpoint("deployments", ec =>
                {
                    ec.ConfigureConsumeTopology = false;
                    ec.Bind(nameof(DeployPlanMessage), b =>
                    {
                        b.RoutingKey = $"{mwConfig.InstanceName}-{mwConfig.InstanceType}";
                        b.ExchangeType = ExchangeType.Direct;
                    });

                    ec.ConfigureConsumer<DeployPlanConsumer>(busRegistrationContext);
                });

                mqBusFactoryConfigurator.ConfigureEndpoints(busRegistrationContext);
            });
        });
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
        return services;
    }
}