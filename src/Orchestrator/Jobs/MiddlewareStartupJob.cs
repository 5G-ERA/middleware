﻿using k8s;
using k8s.Autorest;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Exceptions;
using Quartz;
using Middleware.Common.Job;
using Microsoft.Extensions.Options;
using Middleware.Models.Enums;

namespace Middleware.Orchestrator.Jobs;

public class MiddlewareStartupJob : BaseJob<MiddlewareStartupJob>
{
    private readonly IKubernetesObjectBuilder _kubernetesObjectBuilder;
    private readonly IKubernetesBuilder _kubeBuilder;
    private readonly IOptions<GatewayConfig> _gatewayConfig;
    private readonly IOptions<MiddlewareConfig> _middlewareConfig;

    public MiddlewareStartupJob(ILogger<MiddlewareStartupJob> logger, IKubernetesBuilder kubeBuilder, 
        IOptions<GatewayConfig> gatewayConfig, IOptions<MiddlewareConfig> middlewareConfig,
        IKubernetesObjectBuilder kubernetesObjectBuilder) : base(logger)
    {
        _kubeBuilder = kubeBuilder;
        _kubernetesObjectBuilder = kubernetesObjectBuilder;
        _gatewayConfig = gatewayConfig ?? throw new ArgumentNullException(nameof(gatewayConfig));
        _middlewareConfig = middlewareConfig ?? throw new ArgumentNullException(nameof(middlewareConfig));
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        Logger.LogInformation("Executed");
        try
        {
            var client = _kubeBuilder.CreateKubernetesClient();
            if (client is null)
            {
                Logger.LogInformation("Skipped instantiation of the middleware. Kubernetes not detected.");
                return;
            }

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

                var deployment = _kubernetesObjectBuilder.CreateStartupDeployment(service, tag);

                if (deploymentNames.Contains(deployment.Metadata.Name) == false)
                {
                    deployment = await kubeClient.AppsV1.CreateNamespacedDeploymentAsync(deployment,
                        AppConfig.K8SNamespaceName,
                        shouldDryRun ? "All" : null);
                }
                var nodePort = _gatewayConfig.Value.NodePort;
                var isIngress = _gatewayConfig.Value.IsIngress;
                var instanceType = _middlewareConfig.Value.InstanceType;
                bool useNodePort = false;

                K8SServiceKind kind;
                if(service != "gateway") { kind = K8SServiceKind.ClusterIp; }
                else { 
                    if(isIngress) 
                    {
                        kind = K8SServiceKind.ClusterIp; 
                    }
                    else if(instanceType == LocationType.Edge.ToString())
                    {
                        kind = K8SServiceKind.NodePort;
                        if (nodePort >= 30000 && nodePort <= 32767) { useNodePort = true; }
                    } 
                    else
                    {
                        kind = K8SServiceKind.LoadBalancer;
                    }
                }

                var lbService = _kubernetesObjectBuilder.CreateStartupService(service, kind, deployment.Metadata, nodePort: useNodePort ? nodePort : null);
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