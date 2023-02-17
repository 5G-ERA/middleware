﻿using k8s.KubeConfigModels;
using MassTransit;
using MassTransit.RabbitMqTransport.Configuration;
using Middleware.Common.Config;
using Middleware.Common.MessageContracts;
using RabbitMQ.Client;

namespace Middleware.TaskPlanner.ExtensionMethods;

public static class ServiceCollectionExtensions
{
    public static IServiceCollection RegisterRabbitMqPublishers(this IServiceCollection services,
        RabbitMqConfig mqConfig)
    {
        services.AddMassTransit(x =>
        {
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
                mqBusFactoryConfigurator.Send<DeployPlanMessage>(x =>
                {
                    x.UseRoutingKeyFormatter(t => t.Message.DeploymentLocation);
                });
                mqBusFactoryConfigurator.Message<DeployPlanMessage>(x => x.SetEntityName(nameof(DeployPlanMessage)));
                mqBusFactoryConfigurator.Publish<DeployPlanMessage>(x => { x.ExchangeType = ExchangeType.Direct; });

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