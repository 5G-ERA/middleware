﻿using k8s;
using k8s.Models;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.Orchestrator.Deployment;
using Middleware.RedisInterface.Sdk;
using Quartz;
using Middleware.Common.Job;

namespace Middleware.Orchestrator.Jobs;

[DisallowConcurrentExecution]
public class UpdateStatusJob : BaseJob<UpdateStatusJob>
{
    private readonly IKubernetesBuilder _kubeBuilder;
    private readonly MiddlewareConfig _middlewareConfig;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public UpdateStatusJob(IKubernetesBuilder kubeBuilder,
        ILogger<UpdateStatusJob> logger,
        IConfiguration configuration,
        IRedisInterfaceClient redisInterfaceClient) : base(logger)
    {
        _kubeBuilder = kubeBuilder;
        _redisInterfaceClient = redisInterfaceClient;
        _middlewareConfig = configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        try
        {
            var sequences = await _redisInterfaceClient.ActionPlanGetAllAsync();

            if (sequences is null) return;

            var kubeClient = _kubeBuilder.CreateKubernetesClient();
            if (kubeClient is null)
            {
                Logger.LogInformation("Skipped resource check. Kubernetes not detected.");
                return;
            }

            foreach (var seq in sequences)
            {
                try
                {
                    if (SequenceIsNotRunning(seq))
                    {
                        Logger.LogInformation("Deleting inactive ActionPlan");
                        await _redisInterfaceClient.ActionPlanDeleteAsync(seq.Id);
                        continue;
                    }

                    var updatedSeq = await ValidateSequenceStatusAsync(seq, kubeClient);

                    if (updatedSeq is null) continue;

                    await _redisInterfaceClient.ActionPlanAddAsync(updatedSeq);
                }
                catch (Exception ex)
                {
                    Logger.LogError(ex,
                        "There was en error while updating the status of the action-plan: {Id}, actionPlan: {ActionPlan}",
                        seq.Id, seq);
                }
            }
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Where was a problem during the execution of {Job}", nameof(UpdateStatusJob));
            throw;
        }
    }

    private bool SequenceIsNotRunning(ActionPlanModel seq)
    {
        var statuses = seq.ActionSequence!
            .SelectMany(a => a.Services.Select(s => s.CanBeDeleted())).ToList();

        return statuses.All(b => b);
    }

    private async Task<ActionPlanModel> ValidateSequenceStatusAsync(ActionPlanModel seq, IKubernetes kubeClient)
    {
        var validated = false;
        //check if all instances are down for at least half an hour, then terminate
        foreach (var action in seq.ActionSequence!)
        {
            if (action.Placement != _middlewareConfig.InstanceName ||
                action.PlacementType != _middlewareConfig.InstanceType)
                continue;

            foreach (var instance in action.Services)
            {
                validated = true;
                var deployments = await kubeClient.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName,
                    labelSelector: KubernetesObjectExtensions.GetNetAppLabelSelector(instance.ServiceInstanceId));

                ValidateDeploymentStatus(deployments, instance);

                var services = await kubeClient.CoreV1.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName,
                    labelSelector: KubernetesObjectExtensions.GetNetAppLabelSelector(instance.ServiceInstanceId));

                UpdateServiceStatus(services, instance);
            }
        }

        return validated ? seq : null;
    }

    private void ValidateDeploymentStatus(V1DeploymentList deployments, InstanceModel instance)
    {
        var tmpStatus = ServiceStatus.Down;
        var deployment = deployments.Items.FirstOrDefault();

        if (deployment is null)
        {
            if (instance.ServiceStatus != tmpStatus.ToString())
                instance.SetStatus(tmpStatus);
            return;
        }

        var deploymentStatus = deployment.Status;

        if (deploymentStatus.Replicas.HasValue == false)
        {
            instance.SetStatus(ServiceStatus.Down);
            return;
        }

        if (deploymentStatus.Replicas.Value == default) tmpStatus = ServiceStatus.Terminating;

        if (deploymentStatus.Replicas == deploymentStatus.AvailableReplicas)
            tmpStatus = ServiceStatus.Active;
        else if (deploymentStatus.Replicas > deploymentStatus.AvailableReplicas)
            tmpStatus = ServiceStatus.Instantiating;

        instance.SetStatus(tmpStatus);
    }

    private void UpdateServiceStatus(V1ServiceList services, InstanceModel instance)
    {
        // BB 03.07.2023 - we don't update the addresses of ROS-based services
        // as they're always routed through gateway and do not need to change it's address
        if (instance.RosDistro is not null) return;
        var service = services.Items.FirstOrDefault();
        if (service is null) return;

        var address = service.GetExternalAddress(Logger);
        if (string.IsNullOrEmpty(address) == false)
            instance.ServiceUrl = address;
    }
}