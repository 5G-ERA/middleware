using AutoMapper;
using k8s;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Models;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Deployment;
using Quartz;

namespace Middleware.Orchestrator.Jobs;

public class UpdateStatusJob : BaseJob<UpdateStatusJob>
{
    private readonly IMapper _mapper;
    private readonly RedisInterface.RedisApiClient _redisApiClient;
    private readonly IKubernetes _kubeClient;

    public UpdateStatusJob(IKubernetesBuilder kubeBuilder, IApiClientBuilder apiBuilder, IMapper mapper, ILogger<UpdateStatusJob> logger) : base(logger)
    {
        _mapper = mapper;
        _redisApiClient = apiBuilder.CreateRedisApiClient();
        _kubeClient = kubeBuilder.CreateKubernetesClient();
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        ICollection<RedisInterface.ActionPlanModel> riSequences = await _redisApiClient.ActionPlanGetAllAsync();
        var sequences = _mapper.Map<List<ActionPlanModel>>(riSequences);

        foreach (var seq in sequences)
        {
            try
            {
                var updatedSeq = await ValidateSequenceStatusAsync(seq);

                var riSeq = _mapper.Map<RedisInterface.ActionPlanModel>(updatedSeq);
                await _redisApiClient.ActionPlanAddAsync(riSeq);
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "There was en error while updating the status of the action-plan: {id}, actionPlan: {actionPlan}", seq.Id, seq);
            }
        }
    }

    private async Task<ActionPlanModel> ValidateSequenceStatusAsync(ActionPlanModel seq)
    {
        foreach (var action in seq.ActionSequence)
        {
            foreach (var instance in action.Services)
            {
                var instanceId = instance.ServiceInstanceId;

                var deployments = await _kubeClient.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName,
                    labelSelector: V1ObjectMetaExtensions.GetServiceLabelSelector(instance.ServiceInstanceId));
                var services = await _kubeClient.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName,
                    labelSelector: V1ObjectMetaExtensions.GetServiceLabelSelector(instance.ServiceInstanceId));
            }
        }

        return seq;
    }
}