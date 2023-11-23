using MassTransit;
using Middleware.Common.Config;
using Middleware.Common.Helpers;
using Middleware.ResourcePlanner.Handlers;

namespace Middleware.ResourcePlanner.ExtensionMethods;

public static class ServiceCollectionExtensions
{
    public static IServiceCollection RegisterRabbitMqConsumers(this IServiceCollection services,
        RabbitMqConfig mqConfig, MiddlewareConfig mwConfig)
    {
        services.AddMassTransit(x =>
        {
            services.AddScoped<RequestResourcePlanHandler>();
            x.AddConsumer<RequestResourcePlanHandler>();

            x.UsingRabbitMq((busRegistrationContext, mqBusFactoryConfigurator) =>
            {
                mqBusFactoryConfigurator.Host(mqConfig.Address, "/", hostConfig =>
                {
                    hostConfig.Username(mqConfig.User);
                    hostConfig.Password(mqConfig.Pass);
                });

                mqBusFactoryConfigurator.ReceiveEndpoint(
                    QueueHelpers.ConstructResourcePlanningServiceQueueName(mwConfig.Organization,
                        mwConfig.InstanceName),
                    ec =>
                    {
                        ec.ConfigureConsumeTopology = false;
                        ec.ConfigureConsumer<RequestResourcePlanHandler>(busRegistrationContext);
                    });


                mqBusFactoryConfigurator.ConfigureEndpoints(busRegistrationContext);
            });
        });
        return services;
    }
}