using MassTransit;
using Middleware.Common.Config;
using Middleware.Common.Helpers;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.Handlers;
using Middleware.Orchestrator.Handlers.Slice;
using Middleware.Orchestrator.Handlers.Switchover;
using RabbitMQ.Client;

namespace Middleware.Orchestrator.ExtensionMethods;

public static class ServiceCollectionExtensions
{
    public static IServiceCollection RegisterRabbitMqConsumers(this IServiceCollection services,
        RabbitMqConfig mqConfig, MiddlewareConfig mwConfig)
    {
        var routingKey = QueueHelpers.ConstructRoutingKey(mwConfig.InstanceName, mwConfig.InstanceType);
        services.AddMassTransit(x =>
        {
            services.AddScoped<DeployPlanConsumer>();
            x.AddConsumer<DeployPlanConsumer>();

            services.AddScoped<SwitchoverDeleteInstanceConsumer>();
            x.AddConsumer<SwitchoverDeleteInstanceConsumer>();

            services.AddScoped<SwitchoverDeployInstanceConsumer>();
            x.AddConsumer<SwitchoverDeployInstanceConsumer>();

            services.AddScoped<ConnectRobotToSliceConsumer>();
            x.AddConsumer<ConnectRobotToSliceConsumer>();

            x.UsingRabbitMq((busRegistrationContext, mqBusFactoryConfigurator) =>
            {
                mqBusFactoryConfigurator.Host(mqConfig.Address, "/", hostConfig =>
                {
                    hostConfig.Username(mqConfig.User);
                    hostConfig.Password(mqConfig.Pass);
                });

                #region consumer configuration

                mqBusFactoryConfigurator.ReceiveEndpoint(
                    QueueHelpers.ConstructSwitchoverDeleteActionQueueName(mwConfig.Organization,
                        mwConfig.InstanceName),
                    ec =>
                    {
                        ec.ConfigureConsumeTopology = false;
                        ec.Bind(nameof(SwitchoverDeleteAction), b =>
                        {
                            b.ExchangeType = ExchangeType.Direct;
                            b.RoutingKey = routingKey;
                        });
                        ec.ConfigureConsumer<SwitchoverDeleteInstanceConsumer>(busRegistrationContext);
                    });

                mqBusFactoryConfigurator.ReceiveEndpoint(
                    QueueHelpers.ConstructSwitchoverDeployActionQueueName(mwConfig.Organization,
                        mwConfig.InstanceName),
                    ec =>
                    {
                        ec.ConfigureConsumeTopology = false;
                        ec.Bind(nameof(SwitchoverDeployAction), b =>
                        {
                            b.ExchangeType = ExchangeType.Direct;
                            b.RoutingKey = routingKey;
                        });
                        ec.ConfigureConsumer<SwitchoverDeployInstanceConsumer>(busRegistrationContext);
                    });

                mqBusFactoryConfigurator.ReceiveEndpoint(
                    QueueHelpers.ConstructDeploymentQueueName(mwConfig.Organization, mwConfig.InstanceName),
                    ec =>
                    {
                        ec.ConfigureConsumeTopology = false;
                        ec.Bind(nameof(DeployPlanMessage), b =>
                        {
                            b.ExchangeType = ExchangeType.Direct;
                            b.RoutingKey = routingKey;
                        });
                        ec.ConfigureConsumer<DeployPlanConsumer>(busRegistrationContext);
                    });

                mqBusFactoryConfigurator.ReceiveEndpoint(
                    QueueHelpers.ConstructSliceImsiConnectionQueueName(mwConfig.Organization, mwConfig.InstanceName),
                    ec =>
                    {
                        ec.ConfigureConsumeTopology = false;
                        ec.Bind(nameof(ConnectRobotToSliceMessage), b =>
                        {
                            b.ExchangeType = ExchangeType.Direct;
                            b.RoutingKey = routingKey;
                        });
                        ec.ConfigureConsumer<ConnectRobotToSliceConsumer>(busRegistrationContext);
                    });

                #endregion


                #region publisher configuration

                mqBusFactoryConfigurator.Send<GatewayAddNetAppEntryMessage>(topologyConfigurator =>
                {
                    topologyConfigurator.UseRoutingKeyFormatter(t => t.Message.DeploymentLocation);
                });
                mqBusFactoryConfigurator.Message<GatewayAddNetAppEntryMessage>(topologyConfigurator =>
                    topologyConfigurator.SetEntityName(nameof(GatewayAddNetAppEntryMessage)));
                mqBusFactoryConfigurator.Publish<GatewayAddNetAppEntryMessage>(topologyConfigurator =>
                {
                    topologyConfigurator.ExchangeType = ExchangeType.Direct;
                });


                mqBusFactoryConfigurator.Send<GatewayDeleteNetAppEntryMessage>(topologyConfigurator =>
                {
                    topologyConfigurator.UseRoutingKeyFormatter(t => t.Message.DeploymentLocation);
                });
                mqBusFactoryConfigurator.Message<GatewayDeleteNetAppEntryMessage>(topologyConfigurator =>
                    topologyConfigurator.SetEntityName(nameof(GatewayDeleteNetAppEntryMessage)));
                mqBusFactoryConfigurator.Publish<GatewayDeleteNetAppEntryMessage>(topologyConfigurator =>
                {
                    topologyConfigurator.ExchangeType = ExchangeType.Direct;
                });

                #endregion


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