using MassTransit;
using Middleware.Common.Config;
using Middleware.Common.Helpers;
using Middleware.Common.MessageContracts;
using Middleware.OcelotGateway.Handlers;
using Middleware.OcelotGateway.Services;
using RabbitMQ.Client;

namespace Middleware.OcelotGateway.ExtensionMethods;

public static class ServiceCollectionExtensions
{
    public static IServiceCollection RegisterRabbitMqConsumers(this IServiceCollection services,
        RabbitMqConfig mqConfig, MiddlewareConfig mwConfig)
    {
        var routingKey = QueueHelpers.ConstructRoutingKey(mwConfig.InstanceName, mwConfig.InstanceType);

        services.AddMassTransit(x =>
        {
            services.AddScoped<CreateDynamicRouteConsumer>();
            x.AddConsumer<CreateDynamicRouteConsumer>();

            services.AddScoped<DeleteDynamicRouteConsumer>();
            x.AddConsumer<DeleteDynamicRouteConsumer>();

            x.UsingRabbitMq((busRegistrationContext, mqBusFactoryConfigurator) =>
            {
                mqBusFactoryConfigurator.Host(mqConfig.Address, "/", hostConfig =>
                {
                    hostConfig.Username(mqConfig.User);
                    hostConfig.Password(mqConfig.Pass);
                });

                mqBusFactoryConfigurator.ReceiveEndpoint(
                    QueueHelpers.ConstructGatewayAddNetAppEntryMessageQueueName(mwConfig.Organization, mwConfig.InstanceName),
                    ec =>
                    {
                        ec.ConfigureConsumeTopology = false;
                        ec.Bind(nameof(GatewayAddNetAppEntryMessage), b =>
                        {
                            b.ExchangeType = ExchangeType.Direct;
                            b.RoutingKey = routingKey;
                        });
                        ec.ConfigureConsumer<CreateDynamicRouteConsumer>(busRegistrationContext);
                    });

                mqBusFactoryConfigurator.ReceiveEndpoint(
                    QueueHelpers.ConstructGatewayDeleteNetAppEntryMessageQueueName(mwConfig.Organization, mwConfig.InstanceName),
                    ec =>
                    {
                        ec.ConfigureConsumeTopology = false;
                        ec.Bind(nameof(GatewayDeleteNetAppEntryMessage), b =>
                        {
                            b.ExchangeType = ExchangeType.Direct;
                            b.RoutingKey = routingKey;
                        });
                        ec.ConfigureConsumer<DeleteDynamicRouteConsumer>(busRegistrationContext);
                    });

                mqBusFactoryConfigurator.ConfigureEndpoints(busRegistrationContext);
            });


        });


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
