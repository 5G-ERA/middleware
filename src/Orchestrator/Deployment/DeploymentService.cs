using k8s;
using k8s.Autorest;
using k8s.Models;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Responses;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.Orchestrator.Exceptions;
using Middleware.Orchestrator.Models;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;

namespace Middleware.Orchestrator.Deployment;

internal class DeploymentService : IDeploymentService
{
    private readonly IKubernetesObjectBuilder _kubeObjectBuilder;

    /// <summary>
    ///     Kubernetes client connection builder
    /// </summary>
    private readonly IKubernetesBuilder _kubernetesBuilder;

    /// <summary>
    ///     Logger instance
    /// </summary>
    private readonly ILogger _logger;

    private readonly IOptions<MiddlewareConfig> _mwConfig;

    /// <summary>
    ///     Redis Interface API client
    /// </summary>
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public DeploymentService(IKubernetesBuilder kubernetesBuilder,
        ILogger<DeploymentService> logger,
        IRedisInterfaceClient redisInterfaceClient,
        IOptions<MiddlewareConfig> mwConfig,
        IKubernetesObjectBuilder kubeObjectBuilder)
    {
        _kubernetesBuilder = kubernetesBuilder;
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
        _mwConfig = mwConfig;
        _kubeObjectBuilder = kubeObjectBuilder;
    }

    /// <inheritdoc />
    public async Task<bool> DeployAsync(TaskModel task, Guid robotId)
    {
        var robotResponse = await _redisInterfaceClient.RobotGetByIdAsync(robotId);

        var robot = robotResponse.ToRobot();
        var isSuccess = true;
        try
        {
            var k8SClient = _kubernetesBuilder.CreateKubernetesClient();
            _logger.LogDebug("Entered DeploymentService.DeployAsync");
            var deployments = await k8SClient.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
            var deploymentNames = deployments.Items.Select(d => d.Metadata.Name).OrderBy(d => d).ToList();
            _logger.LogDebug("Current deployments: {deployments}", string.Join(", ", deploymentNames));

            BaseModel location = _mwConfig.Value.InstanceType.ToLower() == "cloud"
                ? (await _redisInterfaceClient.GetCloudByNameAsync(_mwConfig.Value.InstanceName)).ToCloud()
                : (await _redisInterfaceClient.GetEdgeByNameAsync(_mwConfig.Value.InstanceName)).ToEdge();

            foreach (var seq in task.ActionSequence!)
            foreach (var service in seq.Services)
            {
                try
                {
                    // BB: service can be reused, to be decided by the resource planner
                    if (service.ServiceInstanceId != Guid.Empty)
                        continue;

                    await DeployInstance(k8SClient, service, deploymentNames);
                    await _redisInterfaceClient.AddRelationAsync(service, location, "LOCATED_AT");
                }
                catch (HttpOperationException ex)
                {
                    _logger.LogError(ex, "There was an error while deploying hte service {service} caused by {reason}",
                        service.Name, ex.Response.Content);
                    isSuccess = false;
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "There was an error while deploying the service {service}", service.Name);
                    isSuccess = false;
                }
            }

            isSuccess &= await SaveActionSequence(task, robot);
        }
        catch (RedisInterface.ApiException<ApiResponse> apiEx)
        {
            _logger.LogError(apiEx, "There was an error while retrieving the information from Redis");
            isSuccess = false;
        }
        catch (NotInK8SEnvironmentException)
        {
            _logger.LogInformation("The instantiation of the kubernetes client has failed in {env} environment.",
                AppConfig.AppConfiguration);

            isSuccess = AppConfig.AppConfiguration == AppVersionEnum.Dev.GetStringValue();

            if (isSuccess)
            {
                isSuccess &= await SaveActionSequence(task, robot);
                _logger.LogWarning("Deployment of the services has been skipped in the Development environment");
            }
        }

        return isSuccess;
    }

    /// <inheritdoc />
    public V1Service CreateStartupService(string serviceImageName, K8SServiceKind kind, V1ObjectMeta meta)
    {
        return _kubeObjectBuilder.CreateStartupService(serviceImageName, kind, meta);
    }

    /// <inheritdoc />
    public async Task<bool> DeletePlanAsync(ActionPlanModel actionPlan)
    {
        var retVal = true;
        try
        {
            var k8SClient = _kubernetesBuilder.CreateKubernetesClient();
            foreach (var action in actionPlan.ActionSequence!)
            foreach (var srv in action.Services)
            {
                retVal &= await DeleteInstance(k8SClient, srv.ServiceInstanceId);
            }
        }
        catch (NotInK8SEnvironmentException)
        {
            _logger.LogInformation("The instantiation of the kubernetes client has failed in {env} environment.",
                AppConfig.AppConfiguration);

            retVal = AppConfig.AppConfiguration == AppVersionEnum.Dev.GetStringValue();
            if (retVal)
                _logger.LogWarning("Deployment of the services has been skipped in the Development environment");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "The deletion of the Action Plan has failed!");
            retVal = false;
        }

        return retVal;
    }

    public async Task DeleteActionAsync(Guid actionPlanId, Guid actionId)
    {
        _logger.LogTrace("Entered DeployActionAsync");
        var kubeClient = _kubernetesBuilder.CreateKubernetesClient();
        _logger.LogDebug("Retrieving ActionPlan");
        var actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(actionPlanId);
        if (actionPlan is null)
            return;
        _logger.LogDebug("Retrieving Action from action plan");
        var action = actionPlan.ActionSequence!.FirstOrDefault(x => x.Id == actionId);
        if (action is null)
            return;

        _logger.LogDebug("Retrieving location details (cloud or edge)");
        BaseModel thisLocation = _mwConfig.Value.InstanceType == LocationType.Cloud.ToString()
            ? (await _redisInterfaceClient.GetCloudByNameAsync(_mwConfig.Value.InstanceName)).ToCloud()
            : (await _redisInterfaceClient.GetEdgeByNameAsync(_mwConfig.Value.InstanceName)).ToEdge();

        foreach (var instance in action.Services)
        {
            _logger.LogDebug("Deleting instance '{instanceName}', with serviceInstanceId '{serviceInstanceId}'",
                instance.Name, instance.ServiceInstanceId);

            await DeleteInstance(kubeClient, instance.ServiceInstanceId);
            _logger.LogDebug("Deleting relation between instance '{instanceName}'and location '{locationName}'",
                instance.Name, thisLocation.Name);

            await _redisInterfaceClient.DeleteRelationAsync(instance, thisLocation, "LOCATED_AT");
        }
    }

    public async Task DeployActionAsync(Guid actionPlanId, Guid actionId)
    {
        _logger.LogTrace("Entered DeployActionAsync");
        var kubeClient = _kubernetesBuilder.CreateKubernetesClient();

        _logger.LogDebug("Retrieving ActionPlan");
        var actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(actionPlanId);
        if (actionPlan is null)
            return;
        _logger.LogDebug("Retrieving action containing desired Service");
        var action = actionPlan.ActionSequence!.FirstOrDefault(a => a.Id == actionId);

        if (action is null)
            return;

        var deployments = await kubeClient.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
        var deploymentNames = deployments.Items.Select(d => d.Metadata.Name).OrderBy(d => d).ToList();

        _logger.LogDebug("Retrieving location details (cloud or edge)");
        BaseModel thisLocation = _mwConfig.Value.InstanceType == LocationType.Cloud.ToString()
            ? (await _redisInterfaceClient.GetCloudByNameAsync(_mwConfig.Value.InstanceName)).ToCloud()
            : (await _redisInterfaceClient.GetEdgeByNameAsync(_mwConfig.Value.InstanceName)).ToEdge();

        //var instanceNames = action.Services.Select(s => s.Name);
        // evaluate potential addresses 
        // how the environment variables should be named?
        // pass the addresses to the deployment 
        // always expose service through NodePort?

        foreach (var instance in action.Services)
        {
            _logger.LogDebug("Deploying instance '{Name}', with serviceInstanceId '{ServiceInstanceId}'",
                instance.Name, instance.ServiceInstanceId);
            await DeployInstance(kubeClient, instance, deploymentNames);

            _logger.LogDebug("Adding new relation between instance and current location");
            await _redisInterfaceClient.AddRelationAsync(instance, thisLocation, "LOCATED_AT");
        }

        action.Placement = _mwConfig.Value.InstanceName;
        action.PlacementType = _mwConfig.Value.InstanceType;

        _logger.LogDebug("Saving updated ActionPlan");
        await _redisInterfaceClient.ActionPlanAddAsync(actionPlan);
    }

    /// <inheritdoc />
    public V1Deployment CreateStartupDeployment(string name, string tag)
    {
        return _kubeObjectBuilder.CreateStartupDeployment(name, tag);
    }

    /// <summary>
    ///     Saves the specified task to the redis as an action plan
    /// </summary>
    /// <param name="task"></param>
    /// <param name="robot"></param>
    /// <returns></returns>
    private async Task<bool> SaveActionSequence(TaskModel task, RobotModel robot)
    {
        var actionPlan = new ActionPlanModel(task.ActionPlanId, task.Id, task.Name, task.ActionSequence!, robot.Id);
        actionPlan.SetStatus("active");

        var result = await _redisInterfaceClient.ActionPlanAddAsync(actionPlan);
        //await _redisInterfaceClient.AddRelationAsync(robot, actionPlan, "OWNS");
        return result;
    }

    private async Task DeployInstance(IBasicKubernetes k8SClient, InstanceModel service, List<string> deploymentNames)
    {
        _logger.LogDebug("Querying for redis for service {Id}", service.Id);
        var imagesResponse = await _redisInterfaceClient.ContainerImageGetForInstanceAsync(service.Id);

        var images = imagesResponse.ToContainersList();
        if (images is null || images.Any() == false)
            throw new IncorrectDataException("Image is not defined for the Instance deployment");


        _logger.LogDebug("Retrieved service with Id: {Id}", service.Id);

        // usually should only be just one image
        foreach (var cim in images)
        {
            _logger.LogDebug("Deploying the image {ImageName}", service.Name);

            if (deploymentNames.Contains(cim.Name))
            {
                var guidSuffix = Guid.NewGuid().ToString().Split('-')[0];
                cim.Name = $"{cim.Name}-{guidSuffix}";
            }

            deploymentNames.Add(cim.Name);
            var deployedPair = await Deploy(k8SClient, cim, service.Name);

            service.SetStatus(ServiceStatus.Idle);
            service.ServiceInstanceId = deployedPair.InstanceId;
            _logger.LogDebug("Deployed the image {Name} with the Id {ServiceInstanceId}", service.Name,
                service.ServiceInstanceId);
        }
    }

    /// <summary>
    ///     Deploy the service based on the container information
    /// </summary>
    /// <param name="k8SClient"></param>
    /// <param name="cim"></param>
    /// <param name="instanceName"></param>
    /// <returns></returns>
    private async Task<DeploymentPairModel> Deploy(IBasicKubernetes k8SClient, ContainerImageModel cim,
        string instanceName)
    {
        var instanceId = Guid.NewGuid();

        var deployment =
            _kubeObjectBuilder.SerializeAndConfigureDeployment(cim.K8SDeployment, instanceId, instanceName);


        var service = string.IsNullOrWhiteSpace(cim.K8SService)
            ? _kubeObjectBuilder.CreateDefaultService(instanceName, instanceId, deployment)
            : _kubeObjectBuilder.SerializeAndConfigureService(cim.K8SService, instanceName, instanceId);

        service = await k8SClient.CoreV1.CreateNamespacedServiceAsync(service, AppConfig.K8SNamespaceName);
        deployment = await k8SClient.AppsV1.CreateNamespacedDeploymentAsync(deployment, AppConfig.K8SNamespaceName);

        return new(deployment, service, instanceId);
    }

    /// <summary>
    ///     Deletes the instance specified. The deletion includes associated deployment and service
    /// </summary>
    /// <param name="kubeClient"></param>
    /// <param name="instanceId"></param>
    /// <returns></returns>
    private async Task<bool> DeleteInstance(IBasicKubernetes kubeClient, Guid instanceId)
    {
        const string success = "Success";
        var retVal = true;

        var deployments = await kubeClient.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName,
            labelSelector: V1ObjectExtensions.GetNetAppLabelSelector(instanceId));
        var services = await kubeClient.CoreV1.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName,
            labelSelector: V1ObjectExtensions.GetNetAppLabelSelector(instanceId));

        foreach (var deployment in deployments.Items)
        {
            var status =
                await kubeClient.AppsV1.DeleteNamespacedDeploymentAsync(deployment.Name(), AppConfig.K8SNamespaceName);
            retVal &= status.Status == success;
        }

        foreach (var service in services.Items)
        {
            try
            {
                await kubeClient.CoreV1.DeleteNamespacedServiceAsync(service.Name(), AppConfig.K8SNamespaceName);
            }
            catch
            {
                // ignored
            }
        }

        return retVal;
    }
}