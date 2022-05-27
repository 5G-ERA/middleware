using AutoMapper;
using k8s;
using k8s.Models;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Models;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Exceptions;
using Middleware.Orchestrator.Models;

namespace Middleware.Orchestrator.Deployment;

public class DeploymentService : IDeploymentService
{
    /// <summary>
    /// Kubernetes client connection builder
    /// </summary>
    private readonly IKubernetesBuilder _kubernetesBuilder;
    /// <summary>
    /// Environments access to the configuration of a pod
    /// </summary>
    private readonly IEnvironment _env;
    /// <summary>
    /// Object mapper
    /// </summary>
    private readonly IMapper _mapper;
    /// <summary>
    /// Logger instance
    /// </summary>
    private readonly ILogger _logger;
    /// <summary>
    /// Redis Interface client allowing to make calls to the Redis Cache
    /// </summary>
    private readonly RedisInterface.RedisApiClient _redisClient;
    /// <summary>
    /// Name of the AWS registry used 
    /// </summary>
    private readonly string _awsRegistryName;

    public DeploymentService(IKubernetesBuilder kubernetesBuilder, IApiClientBuilder apiClientBuilder, IEnvironment env, IMapper mapper, ILogger<DeploymentService> logger)
    {
        _kubernetesBuilder = kubernetesBuilder;
        _env = env;
        _mapper = mapper;
        _logger = logger;
        _redisClient = apiClientBuilder.CreateRedisApiClient();
        _awsRegistryName = _env.GetEnvVariable("AWS_IMAGE_REGISTRY");
    }

    /// <inheritdoc/>
    public async Task<bool> DeployAsync(TaskModel task)
    {
        bool isSuccess = true;
        try
        {
            var k8SClient = _kubernetesBuilder.CreateKubernetesClient();
            _logger.LogDebug("Entered DeploymentService.DeployAsync");
            var deployments = await k8SClient.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
            var deploymentNames = deployments.Items.Select(d => d.Metadata.Name).OrderBy(d => d).ToArray();
            _logger.LogDebug("Current deployments: {deployments}", string.Join(", ", deploymentNames));

            foreach (var seq in task.ActionSequence)
            {
                foreach (var service in seq.Services)
                {
                    try
                    {
                        await DeployService(k8SClient, service, deploymentNames);
                    }
                    catch (Exception ex)
                    {
                        _logger.LogError(ex, "There was an error while deploying the service");
                        isSuccess = false;
                    }
                }
            }

            isSuccess &= await SaveActionSequence(task);
        }
        catch (RedisInterface.ApiException<ApiResponse> apiEx)
        {
            _logger.LogError(apiEx, "There was an error while retrieving the information from Redis");
            isSuccess = false;
        }
        catch (NotInK8SEnvironmentException)
        {
            _logger.LogInformation("The instantiation of the kubernetes client has failed in {env} environment.", AppConfig.AppConfiguration);

            isSuccess = AppConfig.AppConfiguration == AppVersionEnum.Dev.GetStringValue();

            if (isSuccess)
            {
                isSuccess &= await SaveActionSequence(task);
                _logger.LogWarning("Deployment of the services has been skipped in the Development environment");
            }
        }

        return isSuccess;
    }

    /// <summary>
    /// Saves the specified task to the redis as an action plan
    /// </summary>
    /// <param name="task"></param>
    /// <returns></returns>
    private async Task<bool> SaveActionSequence(TaskModel task)
    {
        var actionPlan = new ActionPlanModel(task.ActionPlanId, task.Name, task.ActionSequence);

        var riActionPlan = _mapper.Map<RedisInterface.ActionPlanModel>(actionPlan);
        var result = await _redisClient.ActionPlanAddAsync(riActionPlan);

        return result != null;
    }

    private async Task DeployService(IKubernetes k8SClient, InstanceModel service, string[] deploymentNames)
    {
        _logger.LogDebug("Querying for redis for service {Id}", service.Id);
        var images = (await _redisClient.ContainerImageGetForInstanceAsync(service.Id))?.ToList();

        if (images is null || images.Any() == false)
        {
            throw new IncorrectDataException("Image is not defined for the Instance deployment");
        }

        var mappedImages = _mapper.Map<List<ContainerImageModel>>(images);

        _logger.LogDebug("Retrieved service with Id: {Id}", service.Id);

        // usually should only be just one image
        foreach (var cim in mappedImages)
        {
            _logger.LogDebug("Deploying the image {ImageName}", service.Name);

            if (deploymentNames.Contains(cim.Name))
            {

                //TODO: handle the check if the deployment or a service already exists
                continue;
            }

            var deployedPair = await Deploy(k8SClient, cim);

            service.ServiceStatus = ServiceStatusEnum.Idle.GetStringValue();
            service.ServiceInstanceId = Guid.Parse(deployedPair.Deployment.GetLabel("serviceId"));
            _logger.LogDebug("Deployed the image {Name} with the Id {ServiceInstanceId}", service.Name,
                service.ServiceInstanceId);

            //TODO: assign values to the instance data
        }
    }

    /// <summary>
    /// Deploy the service based on the container information
    /// </summary>
    /// <param name="k8SClient"></param>
    /// <param name="cim"></param>
    /// <returns></returns>
    public async Task<DeploymentPairModel> Deploy(IKubernetes k8SClient, ContainerImageModel cim)
    {
        var instanceId = Guid.NewGuid();

        var service = await Deploy<V1Service>(k8SClient, cim.K8SService, cim.Name, instanceId, nameof(cim.K8SService));

        var deployment = await Deploy<V1Deployment>(k8SClient, cim.K8SDeployment, cim.Name, instanceId, nameof(cim.K8SDeployment));

        return new DeploymentPairModel(deployment, service);
    }

    /// <summary>
    /// Deploy the Service of the specified type. Available types are <seealso cref="V1Service"/> and <seealso cref="V1Deployment"/>
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
        var type = typeof(T);

        if (string.IsNullOrWhiteSpace(objectDefinition))
        {
            _logger.LogInformation("Definition for {ObjectName} - {Name} has not been specified, the instantiation of the service has been skipped", type.Name, name);

            if (type == typeof(V1Deployment))
                throw new IncorrectDataException($"Deployment for {name} not set");

            return default;
        }
        var sanitized = objectDefinition.SanitizeAsK8SYaml();
        T obj = Yaml.LoadFromString<T>(sanitized);

        if (obj == null)
        {
            throw new UnableToParseYamlConfigException(name, propName);
        }

        if (obj is V1Service srv)
        {
            srv.Metadata.SetServiceLabel(instanceId);

            obj = await k8SClient.CreateNamespacedServiceAsync(srv, AppConfig.K8SNamespaceName) as T;
            return obj;
        }
        if (obj is V1Deployment depl)
        {
            depl.Metadata.SetServiceLabel(instanceId);
            foreach (var container in depl.Spec.Template.Spec.Containers)
            {
                var env = new List<V1EnvVar>(container.Env)
                {
                    new ("NETAPP_ID", instanceId.ToString()),
                    new ("MIDDLEWARE_ADDRESS", AppConfig.GetMiddlewareAddress()),
                    new ("MIDDLEWARE_REPORT_INTERVAL", 5.ToString())
                };
                container.Env = env;
            }

            obj = await k8SClient.CreateNamespacedDeploymentAsync(depl, AppConfig.K8SNamespaceName) as T;
            return obj;
        }

        return default;

    }


    /// <inheritdoc/>
    public V1Service CreateService(string serviceImageName, K8SServiceKindEnum kind, V1ObjectMeta meta)
    {
        var spec = new V1ServiceSpec()
        {
            Ports = new List<V1ServicePort>()
            {
                new(80, "TCP", "http", null, "TCP", 80),
                new(443, "TCP", "https", null, "TCP", 80)
            },
            Selector = new Dictionary<string, string> { { "app", serviceImageName } },
            Type = kind.GetStringValue(),

        };

        var service = new V1Service()
        {
            Metadata = new V1ObjectMeta()
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

    /// <inheritdoc/>
    public async Task<bool> DeletePlanAsync(ActionPlanModel actionPlan)
    {
        bool retVal = true;
        try
        {
            var k8sClient = _kubernetesBuilder.CreateKubernetesClient();
            foreach (var action in actionPlan.ActionSequence)
            {
                foreach (var srv in action.Services)
                {
                    retVal &= await DeleteInstance(k8sClient, srv);
                }
            }
        }
        catch (NotInK8SEnvironmentException)
        {
            _logger.LogInformation("The instantiation of the kubernetes client has failed in {env} environment.", AppConfig.AppConfiguration);

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
    /// <summary>
    /// Deletes the instance specified. The deletion includes associated deployment and service
    /// </summary>
    /// <param name="k8sClient"></param>
    /// <param name="instance"></param>
    /// <returns></returns>
    private async Task<bool> DeleteInstance(IKubernetes k8sClient, InstanceModel instance)
    {
        const string success = "Success";
        var retVal = true;

        var deployments = await k8sClient.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName,
            labelSelector: V1ObjectMetaExtensions.GetServiceLabelSelector(instance.ServiceInstanceId));
        var services = await k8sClient.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName,
            labelSelector: V1ObjectMetaExtensions.GetServiceLabelSelector(instance.ServiceInstanceId));

        foreach (var deployment in deployments.Items)
        {
            var status = await k8sClient.DeleteNamespacedDeploymentAsync(deployment.Name(), AppConfig.K8SNamespaceName);
            retVal &= status.Status == success;
        }

        foreach (var service in services.Items)
        {
            var status = await k8sClient.DeleteNamespacedDeploymentAsync(service.Name(), AppConfig.K8SNamespaceName);
            retVal &= status.Status == success;
        }

        return retVal;
    }

    /// <inheritdoc/>
    public V1Deployment CreateStartupDeployment(string name)
    {
        var selector = new V1LabelSelector
        {
            MatchLabels = new Dictionary<string, string> { { "app", name } }
        };
        var meta = new V1ObjectMeta()
        {
            Name = name,
            Labels = new Dictionary<string, string>()
            {
                { "app", name }
            }
        };
        var envList = new List<V1EnvVar>
        {
            new ("REDIS_INTERFACE_ADDRESS", $"http://redis-interface-api"),
            new ("ORCHESTRATOR_ADDRESS", $"http://orchestrator-api"),
            new ("TASK_PLANNER_ADDRESS", $"http://task-planner-api"),
            new ("RESOURCE_PLANNER_ADDRESS", $"http://resource-planner-api")
        };
        if (name.Contains("redis") || name == "gateway")
        {
            envList.Add(new V1EnvVar("REDIS_HOSTNAME", _env.GetEnvVariable("REDIS_HOSTNAME")));
            envList.Add(new V1EnvVar("REDIS_PORT", _env.GetEnvVariable("REDIS_PORT")));
        }

        var container = new V1Container()
        {
            Name = name,
            Image = K8SImageHelper.BuildImageName(_awsRegistryName, name, "latest"),
            ImagePullPolicy = AppConfig.AppConfiguration == AppVersionEnum.Prod.GetStringValue() ? "Always" : "IfNotPresent",
            Env = envList,
            Ports = new List<V1ContainerPort>() { new(80), new(433) }
        };

        var podSpec = new V1PodSpec(new List<V1Container>() { container });

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
}