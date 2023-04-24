using k8s;
using k8s.Models;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Orchestrator.Deployment;
using Quartz;

namespace Middleware.Orchestrator.Jobs;

public class RefreshMiddlewareAddressJob : BaseJob<RefreshMiddlewareAddressJob>
{
    private readonly IKubernetesBuilder _builder;

    public RefreshMiddlewareAddressJob(IKubernetesBuilder builder, ILogger<RefreshMiddlewareAddressJob> logger) : base(logger)
    {
        _builder = builder;
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        try
        {
            var kubeClient = _builder.CreateKubernetesClient();
            V1Service gateway = await GetGateway(kubeClient);
            AppConfig.MiddlewareAddress = gateway.GetExternalAddress();
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "An error has occurred while refreshing the address of the Middleware.");
        }
    }

    private async Task<V1Service> GetGateway(IKubernetes kubeClient)
    {
        var services = await kubeClient.CoreV1.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName, labelSelector: "app=gateway");
        return services.Items.SingleOrDefault();
    }
}