using k8s;
using k8s.Autorest;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Exceptions;
using Quartz;

namespace Middleware.Orchestrator.Jobs;

public class MiddlewareStartupJob : BaseJob<MiddlewareStartupJob>
{
    private readonly IDeploymentService _deploymentService;
    private readonly IKubernetesBuilder _kubeBuilder;

    public MiddlewareStartupJob(ILogger<MiddlewareStartupJob> logger, IKubernetesBuilder kubeBuilder,
        IDeploymentService deploymentService) : base(logger)
    {
        _kubeBuilder = kubeBuilder;
        _deploymentService = deploymentService;
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        Logger.LogInformation("Executed");
        try
        {
            var client = _kubeBuilder.CreateKubernetesClient();

            await InstantiateMiddleware(client);
        }
        catch (NotInK8SEnvironmentException ex)
        {
            Logger.LogWarning(ex, "Could not instantiate the Kubernetes Client");
            throw;
        }
    }

    /// <summary>
    ///     Instantiates the pods for the middleware
    /// </summary>
    /// <param name="kubeClient"></param>
    /// <returns></returns>
    private async Task InstantiateMiddleware(IKubernetes kubeClient)
    {
        var success = true;
        var shouldDryRun = AppConfig.IsDevEnvironment();
        try
        {
            var deployments = await kubeClient.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
            var deploymentNames = deployments.Items.Select(d => d.Metadata.Name).ToArray();
            var services = await kubeClient.CoreV1.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName);
            var serviceNames = services.Items.Select(d => d.Metadata.Name).ToArray();
            var images = new List<string>
                { "gateway", "redis-interface-api", "resource-planner-api", "task-planner-api" };

            var orchestratorImage = deployments.Items
                .First(d => d.Metadata.Name == "orchestrator-api").Spec.Template.Spec.Containers.FirstOrDefault()
                !.Image;

            var tag = K8SImageHelper.GetTag(orchestratorImage);

            foreach (var service in images)
            {
                Logger.LogDebug("Started deployment of {service}", service);

                var deployment = _deploymentService.CreateStartupDeployment(service, tag);

                if (deploymentNames.Contains(deployment.Metadata.Name) == false)
                {
                    deployment = await kubeClient.AppsV1.CreateNamespacedDeploymentAsync(deployment,
                        AppConfig.K8SNamespaceName,
                        shouldDryRun ? "All" : null);
                }

                var kind = service != "gateway" ? K8SServiceKindEnum.ClusterIp : K8SServiceKindEnum.LoadBalancer;

                var lbService = _deploymentService.CreateService(service, kind, deployment.Metadata);

                if (serviceNames.Contains(lbService.Metadata.Name) == false)
                {
                    lbService = await kubeClient.CoreV1.CreateNamespacedServiceAsync(lbService,
                        AppConfig.K8SNamespaceName);
                }

                if (service == "gateway")
                {
                    AppConfig.MiddlewareAddress = lbService.GetExternalAddress(Logger);

                    if (string.IsNullOrEmpty(AppConfig.MiddlewareAddress))
                        Logger.LogError("Could not obtain the Gateway Address");
                }
            }
        }
        catch (HttpOperationException httpOperationException)
        {
            var phase = httpOperationException.Response.ReasonPhrase;
            var content = httpOperationException.Response.Content;
            Logger.LogError(httpOperationException, "Unable to deploy the resource to k8s:{phase}, {content}", phase,
                content);
            success = false;
        }

        if (success) Logger.LogInformation("Successfully deployed the Middleware.");
    }
}