using k8s;
using k8s.Models;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.Orchestrator.Exceptions;
using Middleware.Orchestrator.Models;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;

namespace Middleware.Orchestrator.Deployment;

internal class DeploymentService : IDeploymentService
{
    /// <summary>
    ///     Kubernetes client instance
    /// </summary>
    private readonly IKubernetes _kube;

    /// <summary>
    ///     Builds and configures the kubernetes objects
    /// </summary>
    private readonly IKubernetesObjectBuilder _kubeObjectBuilder;

    /// <summary>
    ///     Logger instance
    /// </summary>
    private readonly ILogger _logger;

    private readonly IOptions<MiddlewareConfig> _mwConfig;

    /// <summary>
    ///     Redis Interface API client
    /// </summary>
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    private readonly IRosConnectionBuilderFactory _rosConnectionBuilderFactory;

    public DeploymentService(IKubernetesBuilder kubernetesClientBuilder,
        ILogger<DeploymentService> logger,
        IRedisInterfaceClient redisInterfaceClient,
        IOptions<MiddlewareConfig> mwConfig,
        IKubernetesObjectBuilder kubeObjectBuilder, IRosConnectionBuilderFactory rosConnectionBuilderFactory)
    {
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
        _mwConfig = mwConfig;
        _kubeObjectBuilder = kubeObjectBuilder;
        _rosConnectionBuilderFactory = rosConnectionBuilderFactory;
        _kube = kubernetesClientBuilder.CreateKubernetesClient();
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
            foreach (var action in actionPlan.ActionSequence!)
            foreach (var srv in action.Services)
            {
                retVal &= await TerminateNetAppByIdAsync(srv.ServiceInstanceId);
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
        _logger.LogDebug("Retrieving ActionPlan");
        var actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(actionPlanId);
        if (actionPlan is null)
            return;
        _logger.LogDebug("Retrieving Action from action plan");
        var action = actionPlan.ActionSequence!.FirstOrDefault(x => x.Id == actionId);
        if (action is null)
            return;

        _logger.LogDebug("Retrieving location details (cloud or edge)");
        var thisLocation = await GetCurrentLocationAsync();

        foreach (var instance in action.Services)
        {
            _logger.LogDebug("Deleting instance '{instanceName}', with serviceInstanceId '{serviceInstanceId}'",
                instance.Name, instance.ServiceInstanceId);

            await TerminateNetAppByIdAsync(instance.ServiceInstanceId);
            _logger.LogDebug("Deleting relation between instance '{instanceName}'and location '{locationName}'",
                instance.Name, thisLocation.Name);

            await _redisInterfaceClient.DeleteRelationAsync(instance, thisLocation, "LOCATED_AT");
        }
    }

    public async Task DeployActionAsync(Guid actionPlanId, Guid actionId)
    {
        _logger.LogTrace("Entered DeployActionAsync");
        _logger.LogDebug("Retrieving ActionPlan");
        var actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(actionPlanId);
        if (actionPlan is null)
            return;
        _logger.LogDebug("Retrieving action containing desired Service");
        var action = actionPlan.ActionSequence!.FirstOrDefault(a => a.Id == actionId);

        if (action is null)
            return;

        var deploymentNames = await GetCurrentlyDeployedAppNames();

        var thisLocation = await GetCurrentLocationAsync();

        var deploymentPairs = await ConstructDeployments(action, deploymentNames);

        foreach (var pair in deploymentPairs)
        {
            await DeployNetApp(pair);
            _logger.LogDebug("Adding new relation between instance and current location");
            await _redisInterfaceClient.AddRelationAsync(pair.Instance, thisLocation, "LOCATED_AT");
        }

        _logger.LogDebug("Saving updated ActionPlan");
        await _redisInterfaceClient.ActionPlanAddAsync(actionPlan);
    }

    /// <inheritdoc />
    public V1Deployment CreateStartupDeployment(string name, string tag)
    {
        return _kubeObjectBuilder.CreateStartupDeployment(name, tag);
    }

    public async Task<bool> DeployActionPlanAsync(TaskModel task, Guid robotId)
    {
        if (_kube is null)
            return true;

        var robotResp = await _redisInterfaceClient.RobotGetByIdAsync(robotId);
        var robot = robotResp.ToRobot();

        var isSuccess = true;
        var deploymentQueue = new List<DeploymentPair>();
        try
        {
            _logger.LogDebug("Entered DeploymentService.DeployAsync");
            var deployments = await _kube.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
            var deploymentNames = deployments.GetDeploymentNames().ToList();
            _logger.LogDebug("Current deployments: {deployments}", string.Join(", ", deploymentNames));

            var location = await GetCurrentLocationAsync();

            foreach (var action in task.ActionSequence!)
            {
                var dplTmp = await ConstructDeployments(action, deploymentNames);
                deploymentQueue.AddRange(dplTmp);
            }

            foreach (var pair in deploymentQueue)
            {
                _logger.LogDebug("Deploying instance '{Name}', with serviceInstanceId '{ServiceInstanceId}'",
                    pair.Instance, pair.InstanceId);
                await DeployNetApp(pair);
                _logger.LogDebug("Adding new relation between instance and current location");
                await _redisInterfaceClient.AddRelationAsync(pair.Instance, location, "LOCATED_AT");
            }

            isSuccess &= await SaveActionSequence(task, robot);
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
        catch (Exception ex)
        {
            _logger.LogError(ex, "The deployment of the Action Plan has failed!");
            isSuccess = false;
        }

        return isSuccess;
    }

    public async Task<DeploymentPair> DeployNetApp(DeploymentPair netApp)
    {
        var service = await _kube.CoreV1.CreateNamespacedServiceAsync(netApp.Service, AppConfig.K8SNamespaceName);
        var deployment =
            await _kube.AppsV1.CreateNamespacedDeploymentAsync(netApp.Deployment, AppConfig.K8SNamespaceName);

        return new(deployment, service, netApp.InstanceId, netApp.Instance);
    }

    private async Task<List<string>> GetCurrentlyDeployedAppNames()
    {
        var deployments = await _kube.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
        var deploymentNames = deployments.GetDeploymentNames().ToList();
        _logger.LogDebug("Current deployments: {deployments}", string.Join(", ", deploymentNames));
        return deploymentNames;
    }

    private async Task<IReadOnlyList<DeploymentPair>> ConstructDeployments(ActionModel action,
        ICollection<string> deploymentNames)
    {
        var deployments = new List<DeploymentPair>();
        foreach (var service in action.Services)
        {
            try
            {
                // BB: service can be reused, to be decided by the resource planner
                if (service.ServiceInstanceId != Guid.Empty)
                    continue;
                var pair = await PrepareDeploymentPair(service, deploymentNames);
                deployments.Add(pair);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was an error while preparing service deployment: {service}", service.Name);
            }
        }

        _kubeObjectBuilder.ConfigureCrossNetAppConnection(deployments);

        return deployments;
    }

    private async Task<DeploymentPair> PrepareDeploymentPair(InstanceModel service, ICollection<string> deploymentNames)
    {
        _logger.LogDebug("Querying for the images for service {Id}", service.Id);
        var imagesResponse = await _redisInterfaceClient.ContainerImageGetForInstanceAsync(service.Id);

        var images = imagesResponse.ToContainersList();
        if (images is null || images.Any() == false)
            throw new IncorrectDataException("Image is not defined for the Instance deployment");

        _logger.LogDebug("Retrieved service with Id: {Id}", service.Id);

        var cim = service.ContainerImage = images.First();

        _logger.LogDebug("Preparing the image {ImageName}", service.Name);

        if (deploymentNames.Contains(cim.Name)) cim.Name = GetNewImageNameWithSuffix(cim.Name);

        deploymentNames.Add(cim.Name);
        var pair = ConfigureDeploymentObjects(service);

        service.SetStatus(ServiceStatus.Instantiating);
        service.ServiceInstanceId = pair.InstanceId;

        return pair;
    }

    private string GetNewImageNameWithSuffix(string name)
    {
        var guidSuffix = Guid.NewGuid().ToString().Split('-')[0];
        return $"{name}-{guidSuffix}";
    }

    private async Task<BaseModel> GetCurrentLocationAsync()
    {
        _logger.LogDebug("Retrieving location details (cloud or edge)");
        return _mwConfig.Value.InstanceType.ToLower() == "cloud"
            ? (await _redisInterfaceClient.GetCloudByNameAsync(_mwConfig.Value.InstanceName)).ToCloud()
            : (await _redisInterfaceClient.GetEdgeByNameAsync(_mwConfig.Value.InstanceName)).ToEdge();
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

    private DeploymentPair ConfigureDeploymentObjects(InstanceModel instance)
    {
        var cim = instance.ContainerImage;
        var instanceName = instance.Name;
        var instanceId = Guid.NewGuid();

        var deployment =
            _kubeObjectBuilder.DeserializeAndConfigureDeployment(cim!.K8SDeployment, instanceId, instanceName);

        IRosConnectionBuilder builder = null;

        if (instance.RosDistro is not null)
        {
            var distroEnum = RosDistroHelper.FromName(instance.RosDistro);
            builder = _rosConnectionBuilderFactory.CreateConnectionBuilder(distroEnum);
        }

        if (builder is not null)
        {
            var topics = instance.RosTopicsSub.CreateCopy();
            topics.AddRange(instance.RosTopicsPub);
            deployment = builder.EnableRosCommunication(deployment, topics);

        }

        var service = string.IsNullOrWhiteSpace(cim.K8SService)
            ? _kubeObjectBuilder.CreateDefaultService(instanceName, instanceId, deployment)
            : _kubeObjectBuilder.DeserializeAndConfigureService(cim.K8SService, instanceName, instanceId);

        if (builder is not null) service = builder.EnableRelayNetAppCommunication(service);

        return new(deployment, service, instanceId, instance);
    }

    /// <summary>
    ///     Deletes the instance specified. The deletion includes associated deployment and service
    /// </summary>
    /// <param name="instanceId"></param>
    /// <returns></returns>
    private async Task<bool> TerminateNetAppByIdAsync(Guid instanceId)
    {
        const string success = "Success";
        var retVal = true;

        var deployments = await _kube.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName,
            labelSelector: KubernetesObjectExtensions.GetNetAppLabelSelector(instanceId));
        var services = await _kube.CoreV1.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName,
            labelSelector: KubernetesObjectExtensions.GetNetAppLabelSelector(instanceId));

        foreach (var deployment in deployments.Items)
        {
            var status =
                await _kube.AppsV1.DeleteNamespacedDeploymentAsync(deployment.Name(), AppConfig.K8SNamespaceName);
            retVal &= status.Status == success;
        }

        foreach (var service in services.Items)
        {
            try
            {
                await _kube.CoreV1.DeleteNamespacedServiceAsync(service.Name(), AppConfig.K8SNamespaceName);
            }
            catch
            {
                // ignored
            }
        }

        return retVal;
    }
}