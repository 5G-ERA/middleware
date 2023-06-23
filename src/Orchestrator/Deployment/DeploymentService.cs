using k8s;
using k8s.Autorest;
using k8s.Models;
using Microsoft.Extensions.Options;
using Middleware.Common;
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

public class DeploymentService : IDeploymentService
{
    private readonly IConfiguration _configuration;

    /// <summary>
    ///     Name of the container registry used
    /// </summary>
    private readonly string _containerRegistryName;

    /// <summary>
    ///     Environments access to the configuration of a pod
    /// </summary>
    private readonly IEnvironment _env;

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
        IEnvironment env,
        ILogger<DeploymentService> logger,
        IRedisInterfaceClient redisInterfaceClient,
        IConfiguration configuration,
        IOptions<MiddlewareConfig> mwConfig)
    {
        _kubernetesBuilder = kubernetesBuilder;
        _env = env;
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
        _configuration = configuration;
        _mwConfig = mwConfig;
        _containerRegistryName = _env.GetEnvVariable("IMAGE_REGISTRY") ?? "ghcr.io/5g-era";
    }

    /// <inheritdoc />
    public async Task<bool> DeployAsync(TaskModel task, Guid robotId)
    {
        var robotResposne = await _redisInterfaceClient.RobotGetByIdAsync(robotId);

        var robot = robotResposne.ToRobot();
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
            foreach (var service in seq.Services!)
            {
                try
                {
                    // BB: service can be reused, to be decided by the resource planner
                    if (service.ServiceInstanceId != Guid.Empty)
                        continue;

                    await DeployService(k8SClient, service, deploymentNames);
                    //await _redisInterfaceClient.AddRelationAsync(service, location, "LOCATED_AT");
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

            isSuccess &= await SaveActionSequence(task, robot); //Here saved in index 13
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
    public V1Service CreateService(string serviceImageName, K8SServiceKindEnum kind, V1ObjectMeta meta)
    {
        var spec = new V1ServiceSpec
        {
            Ports = new List<V1ServicePort>
            {
                new(80, "TCP", "http", null, "TCP", 80),
                new(443, "TCP", "https", null, "TCP", 80)
            },
            Selector = new Dictionary<string, string> { { "app", serviceImageName } },
            Type = kind.GetStringValue()
        };

        var service = new V1Service
        {
            Metadata = new()
            {
                Name = meta.Name,
                Labels = meta.Labels
            },
            ApiVersion = "v1",
            Spec = spec,
            Kind = "Service"
        };
        return service;
    }

    /// <inheritdoc />
    public async Task<bool> DeletePlanAsync(ActionPlanModel actionPlan)
    {
        var retVal = true;
        try
        {
            var k8SClient = _kubernetesBuilder.CreateKubernetesClient();
            foreach (var action in actionPlan.ActionSequence)
            foreach (var srv in action.Services!)
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
        var action = actionPlan.ActionSequence.FirstOrDefault(x => x.Id == actionId);
        if (action is null)
            return;

        _logger.LogDebug("Retrieving location details (cloud or edge)");
        BaseModel thisLocation = _mwConfig.Value.InstanceType == LocationType.Cloud.ToString()
            ? (await _redisInterfaceClient.GetCloudByNameAsync(_mwConfig.Value.InstanceName)).ToCloud()
            : (await _redisInterfaceClient.GetEdgeByNameAsync(_mwConfig.Value.InstanceName)).ToEdge();

        foreach (var instance in action.Services!)
        {
            _logger.LogDebug("Deleting instance '{0}', with serviceInstanceId '{1}'",
                instance.Name, instance.ServiceInstanceId);

            await DeleteInstance(kubeClient, instance.ServiceInstanceId);
            _logger.LogDebug("Deleting relation between instance '{0}'and location '{1}'",
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
        var action = actionPlan.ActionSequence.FirstOrDefault(a => a.Id == actionId);

        if (action is null)
            return;

        var deployments = await kubeClient.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
        var deploymentNames = deployments.Items.Select(d => d.Metadata.Name).OrderBy(d => d).ToList();

        _logger.LogDebug("Retrieving location details (cloud or edge)");
        BaseModel thisLocation = _mwConfig.Value.InstanceType == LocationType.Cloud.ToString()
            ? (await _redisInterfaceClient.GetCloudByNameAsync(_mwConfig.Value.InstanceName)).ToCloud()
            : (await _redisInterfaceClient.GetEdgeByNameAsync(_mwConfig.Value.InstanceName)).ToEdge();

        foreach (var instance in action.Services!)
        {
            _logger.LogDebug("Deploying instance '{0}', with serviceInstanceId '{1}'",
                instance.Name, instance.ServiceInstanceId);
            await DeployService(kubeClient, instance, deploymentNames);

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
        var mwConfig = _configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
        var selector = new V1LabelSelector
        {
            MatchLabels = new Dictionary<string, string> { { "app", name } }
        };
        var meta = new V1ObjectMeta
        {
            Name = name,
            Labels = new Dictionary<string, string>
            {
                { "app", name }
            }
        };
        var envList = new List<V1EnvVar>
        {
            new("Middleware__Organization", mwConfig.Organization),
            new("Middleware__InstanceName", mwConfig.InstanceName),
            new("Middleware__InstanceType", mwConfig.InstanceType),
            new("CustomLogger__LoggerName", _env.GetEnvVariable("CustomLogger__LoggerName")),
            new("CustomLogger__Url", _env.GetEnvVariable("CustomLogger__Url")),
            new("CustomLogger__User", _env.GetEnvVariable("CustomLogger__User")),
            new("CustomLogger__Password", _env.GetEnvVariable("CustomLogger__Password")),
            new("Slice__Hostname", _env.GetEnvVariable("Slice__Hostname")),
            new("RabbitMQ__Address", _env.GetEnvVariable("RabbitMQ__Address")),
            new("RabbitMQ__User", _env.GetEnvVariable("RabbitMQ__User")),
            new("RabbitMQ__Pass", _env.GetEnvVariable("RabbitMQ__Pass")),
            new("CENTRAL_API_HOSTNAME", _env.GetEnvVariable("CENTRAL_API_HOSTNAME")),
            new("AWS_ACCESS_KEY_ID", _env.GetEnvVariable("AWS_ACCESS_KEY_ID")),
            new("AWS_SECRET_ACCESS_KEY", _env.GetEnvVariable("AWS_SECRET_ACCESS_KEY"))
        };
        if (name.Contains("redis") || name == "gateway")
        {
            envList.Add(new("Redis__HostName", _env.GetEnvVariable("Redis__HostName")));
            envList.Add(new("Redis__ClusterHostname", _env.GetEnvVariable("Redis__ClusterHostname")));
            envList.Add(new("Redis__Password", _env.GetEnvVariable("Redis__Password")));
        }

        var container = new V1Container
        {
            Name = name,
            Image = K8SImageHelper.BuildImageName(_containerRegistryName, name, tag),
            ImagePullPolicy = AppConfig.AppConfiguration == AppVersionEnum.Prod.GetStringValue()
                ? "Always"
                : "IfNotPresent",
            Env = envList,
            Ports = new List<V1ContainerPort> { new(80), new(433) }
        };

        var podSpec = new V1PodSpec(new List<V1Container> { container });

        var template = new V1PodTemplateSpec(meta, podSpec);
        var spec = new V1DeploymentSpec(selector, template);

        var dep = new V1Deployment
        {
            Metadata = meta,
            Spec = spec,
            ApiVersion = "apps/v1",
            Kind = "Deployment"
        };
        return dep;
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

    private async Task DeployService(IKubernetes k8SClient, InstanceModel service, List<string> deploymentNames)
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
    private async Task<DeploymentPairModel> Deploy(IKubernetes k8SClient, ContainerImageModel cim, string instanceName)
    {
        var instanceId = Guid.NewGuid();

        var service =
            await Deploy<V1Service>(k8SClient, cim.K8SService, instanceName, instanceId, nameof(cim.K8SService));

        var deployment = await Deploy<V1Deployment>(k8SClient, cim.K8SDeployment, instanceName, instanceId,
            nameof(cim.K8SDeployment));

        return new(deployment, service, instanceId);
    }

    /// <summary>
    ///     Deploy the Service of the specified type. Available types are <see cref="V1Service" /> and
    ///     <see cref="V1Deployment" />
    /// </summary>
    /// <typeparam name="T">Type of the service</typeparam>
    /// <param name="k8SClient"></param>
    /// <param name="objectDefinition">string representation of the k8s yaml object</param>
    /// <param name="name">Name of the service to be deployed</param>
    /// <param name="instanceId"></param>
    /// <param name="propName"></param>
    /// <returns></returns>
    /// <exception cref="IncorrectDataException"></exception>
    /// <exception cref="UnableToParseYamlConfigException"></exception>
    private async Task<T> Deploy<T>(IKubernetes k8SClient, string objectDefinition, string name, Guid instanceId,
        string propName = null) where T : class
    {
        name = name.Replace(" ", "-")
            .Replace('_', '-')
            .Replace(':', '-')
            .Replace('.', '-')
            .Replace('/', '-')
            .ToLower().Trim();
        var type = typeof(T);

        if (string.IsNullOrWhiteSpace(objectDefinition))
        {
            _logger.LogInformation(
                "Definition for {ObjectName} - {Name} has not been specified, the instantiation of the service has been skipped",
                type.Name, name);

            if (type == typeof(V1Deployment))
                throw new IncorrectDataException($"Deployment for {name} not set");

            return default;
        }

        var sanitized = objectDefinition.SanitizeAsK8SYaml();
        var obj = KubernetesYaml.Deserialize<T>(sanitized);

        if (obj == null) throw new UnableToParseYamlConfigException(name, propName);

        if (obj is V1Service srv)
        {
            srv.Metadata.SetServiceLabel(instanceId);
            srv.Metadata.Name = name;
            obj = await k8SClient.CoreV1.CreateNamespacedServiceAsync(srv, AppConfig.K8SNamespaceName) as T;
            return obj;
        }

        if (obj is V1Deployment deployment)
        {
            deployment.Metadata.SetServiceLabel(instanceId);
            deployment.Metadata.AddNetAppMultusAnnotations(AppConfig.MultusNetworkName);
            deployment.Metadata.Name = name;
            foreach (var container in deployment.Spec.Template.Spec.Containers)
            {
                var envVars = container.Env is not null
                    ? new(container.Env)
                    : new List<V1EnvVar>();

                envVars.Add(new("NETAPP_ID", instanceId.ToString()));
                envVars.Add(new("MIDDLEWARE_ADDRESS", AppConfig.GetMiddlewareAddress()));
                envVars.Add(new("MIDDLEWARE_REPORT_INTERVAL", 5.ToString()));

                container.Env = envVars;
            }

            obj = await k8SClient.AppsV1.CreateNamespacedDeploymentAsync(deployment, AppConfig.K8SNamespaceName) as T;
            return obj;
        }

        return default;
    }

    /// <summary>
    ///     Deletes the instance specified. The deletion includes associated deployment and service
    /// </summary>
    /// <param name="kubeClient"></param>
    /// <param name="instanceId"></param>
    /// <returns></returns>
    private async Task<bool> DeleteInstance(IKubernetes kubeClient, Guid instanceId)
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
            //var version = await kubeClient.Version.GetCodeWithHttpMessagesAsync();
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