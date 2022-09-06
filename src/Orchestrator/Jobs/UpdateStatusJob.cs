﻿using System.Net;
using AutoMapper;
using k8s;
using k8s.Models;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Models;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Deployment;
using Quartz;

namespace Middleware.Orchestrator.Jobs;

[DisallowConcurrentExecution]
public class UpdateStatusJob : BaseJob<UpdateStatusJob>
{
    private readonly IKubernetesBuilder _kubeBuilder;
    private readonly IMapper _mapper;
    private readonly RedisInterface.RedisApiClient _redisApiClient;

    public UpdateStatusJob(IKubernetesBuilder kubeBuilder, IApiClientBuilder apiBuilder, IMapper mapper, ILogger<UpdateStatusJob> logger) : base(logger)
    {
        _kubeBuilder = kubeBuilder;
        _mapper = mapper;
        _redisApiClient = apiBuilder.CreateRedisApiClient();

    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        try
        {
            ICollection<RedisInterface.ActionPlanModel> riSequences = await _redisApiClient.ActionPlanGetAllAsync();
            var sequences = _mapper.Map<List<ActionPlanModel>>(riSequences);
            var kubeClient = _kubeBuilder.CreateKubernetesClient();
            foreach (var seq in sequences)
            {
                try
                {
                    var updatedSeq = await ValidateSequenceStatusAsync(seq, kubeClient);
                    // List<string> statuses = updatedSeq.ActionSequence
                    //     .SelectMany(a => a.Services.Select(s => s.ServiceStatus)).ToList();
                    // if (statuses.Any(s => s != ServiceStatus.Down.GetStringValue()) == false)
                    // {
                    //     await _redisApiClient.ActionPlanDeleteAsync(updatedSeq.Id);
                    //     continue;
                    // }

                    var riSeq = _mapper.Map<RedisInterface.ActionPlanModel>(updatedSeq);
                    await _redisApiClient.ActionPlanAddAsync(riSeq);
                }
                catch (Exception ex)
                {
                    Logger.LogError(ex,
                        "There was en error while updating the status of the action-plan: {Id}, actionPlan: {ActionPlan}",
                        seq.Id, seq);
                }
            }
        }
        catch (RedisInterface.ApiException<RedisInterface.ApiResponse> apiEx)
        {
            if (apiEx.StatusCode == (int)HttpStatusCode.NotFound)
            {
                Logger.LogInformation(apiEx, "No deployed plans have been found");
                return;
            }

            Logger.LogError(apiEx, "There was a problem during the operation on the data");
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Where was a problem during the execution of {Job}", nameof(UpdateStatusJob));
            throw;
        }
    }

    private async Task<ActionPlanModel> ValidateSequenceStatusAsync(ActionPlanModel seq, IKubernetes kubeClient)
    {
        //check if all instances are down for at least half an hour, then terminate
        foreach (var action in seq.ActionSequence)
        {
            foreach (var instance in action.Services)
            {
                var instanceId = instance.ServiceInstanceId;

                var deployments = await kubeClient.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName,
                    labelSelector: V1ObjectExtensions.GetNetAppLabelSelector(instance.ServiceInstanceId));

                ValidateDeploymentStatus(deployments, instance);

                var services = await kubeClient.CoreV1.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName,
                    labelSelector: V1ObjectExtensions.GetNetAppLabelSelector(instance.ServiceInstanceId));

                UpdateServiceStatus(services, instance);
            }
        }

        return seq;
    }

    private void ValidateDeploymentStatus(V1DeploymentList deployments, InstanceModel instance)
    {
        ServiceStatus tmpStatus = ServiceStatus.Down;
        var deployment = deployments.Items.FirstOrDefault();

        if (deployment is null)
        {
            instance.ServiceStatus = tmpStatus.ToString();
            return;
        }
        
        var deploymentStatus = deployment.Status;

        if (deploymentStatus.Replicas.HasValue == false)
        {
            instance.ServiceStatus = ServiceStatus.Down.ToString();
            return;
        }

        if (deploymentStatus.Replicas.Value == default)
        {
            tmpStatus = ServiceStatus.Terminating;
        }

        if (deploymentStatus.Replicas == deploymentStatus.AvailableReplicas)
        {
            tmpStatus = ServiceStatus.Active;
        }
        else if (deploymentStatus.Replicas > deploymentStatus.AvailableReplicas)
        {
            tmpStatus = ServiceStatus.Instantiating;
        }

        instance.ServiceStatus = tmpStatus.GetStringValue();
    }

    private void UpdateServiceStatus(V1ServiceList services, InstanceModel instance)
    {
        var service = services.Items.FirstOrDefault();
        if (service is null)
        {
            return;
        }

        var address = service.GetExternalAddress(Logger);
        if (Uri.IsWellFormedUriString(address, UriKind.Absolute))
        {
            instance.ServiceUrl = new Uri(address);
        }
    }
}