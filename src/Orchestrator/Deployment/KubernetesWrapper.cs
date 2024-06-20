using k8s;
using k8s.Autorest;
using k8s.Models;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Result;
using Middleware.Orchestrator.Helpers;
using Middleware.Orchestrator.Models;

namespace Middleware.Orchestrator.Deployment;

public class KubernetesWrapper : IKubernetesWrapper
{
    private readonly ILogger<KubernetesWrapper> _logger;
    private readonly IKubernetes _kubernetes;

    public KubernetesWrapper(IKubernetesBuilder kubeBuilder, ILogger<KubernetesWrapper> logger)
    {
        _logger = logger;
        _kubernetes = kubeBuilder.CreateKubernetesClient();
    }

    /// <inheritdoc />
    public async Task<List<string>> GetCurrentlyDeployedNetApps()
    {
        var deployments = await _kubernetes.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
        var deploymentNames = deployments.GetDeploymentNames().ToList();
        _logger.LogDebug("Current deployments: {deployments}", string.Join(", ", deploymentNames));
        return deploymentNames;
    }

    /// <inheritdoc />
    public async Task<Result> DeployNetApp(DeploymentPair netApp)
    {
        try
        {
            await _kubernetes.CoreV1.CreateNamespacedServiceAsync(netApp.Service, AppConfig.K8SNamespaceName);
            await _kubernetes.AppsV1.CreateNamespacedDeploymentAsync(netApp.Deployment, AppConfig.K8SNamespaceName);
            return Result.Success();
        }
        catch (HttpOperationException ex)
        {
            _logger.LogError("Failed to deploy NetApp: {reason}", ex.Response.Content);
            return Result.Failure(ex.Response.ReasonPhrase);
        }
    }

    public async Task<Result> TerminateNetAppById(Guid instanceId)
    {
        var deployments = await _kubernetes.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName,
            labelSelector: KubernetesObjectExtensions.GetNetAppLabelSelector(instanceId));
        var services = await _kubernetes.CoreV1.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName,
            labelSelector: KubernetesObjectExtensions.GetNetAppLabelSelector(instanceId));


        return await Terminate(deployments, services);
    }

    public async Task<Result<string>> TerminateInterRelayNetApp(Dictionary<string, string> labels)
    {
        var deployments = await _kubernetes.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName,
            labelSelector: labels.ToLabelSelectorString());
        var services = await _kubernetes.CoreV1.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName,
            labelSelector: labels.ToLabelSelectorString());

        var relayName = deployments.Items.FirstOrDefault()?.Name();
        if (string.IsNullOrEmpty(relayName))
        {
            return Result.Failure("Relay to delete not found");
        }
        
        var terminateResult = await Terminate(deployments, services);
        if (terminateResult.IsFailure)
        {
            return terminateResult;
        }

        await Terminate(deployments, services);

        return Result<string>.Success(relayName);
    }

    private async Task<Result> Terminate(V1DeploymentList deployments, V1ServiceList services)
    {
        var retVal = new Result();

        foreach (var deployment in deployments.Items)
        {
            var status =
                await _kubernetes.AppsV1.DeleteNamespacedDeploymentAsync(deployment.Name(), AppConfig.K8SNamespaceName);
            if (status.Status != OutcomeType.Successful)
                retVal += $"Failed to delete deployment {deployment.Name()}";
        }

        foreach (var service in services.Items)
        {
            try
            {
                await _kubernetes.CoreV1.DeleteNamespacedServiceAsync(service.Name(), AppConfig.K8SNamespaceName);
            }
            catch
            {
                retVal += $"Failed to delete service {service.Name()}";
            }
        }

        return retVal;
    }
}