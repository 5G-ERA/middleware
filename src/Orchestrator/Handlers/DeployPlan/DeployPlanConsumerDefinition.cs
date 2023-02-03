using MassTransit;
using Middleware.Common.Config;

namespace Middleware.Orchestrator.Handlers;

// ReSharper disable once ClassNeverInstantiated.Global
public class DeployPlanConsumerDefinition : ConsumerDefinition<DeployPlanConsumer>
{
    public DeployPlanConsumerDefinition()
    {
        var mwName = AppConfig.MiddlewareDeploymentLocationName;
        // override the default endpoint name
        EndpointName = $"deployments-{mwName}";
    }

    protected override void ConfigureConsumer(IReceiveEndpointConfigurator endpointConfigurator,
        IConsumerConfigurator<DeployPlanConsumer> consumerConfigurator)
    {
        // // configure message retry with millisecond intervals
        // endpointConfigurator.UseMessageRetry(r => r.Intervals(100,200,500,800,1000));
        //
        // // use the outbox to prevent duplicate events from being published
        // endpointConfigurator.UseInMemoryOutbox();
    }
}