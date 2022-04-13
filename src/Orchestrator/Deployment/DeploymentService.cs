using System.Linq;
using k8s;
using k8s.Models;
using Middleware.Common;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Exceptions;
using Middleware.Orchestrator.Models;
using Middleware.Orchestrator.RedisInterface;
using TaskModel = Middleware.Common.Models.TaskModel;

namespace Middleware.Orchestrator.Deployment;

public class DeploymentService : IDeploymentService
{
    /// <summary>
    /// Kubernetes client connection builder
    /// </summary>
    private readonly IKubernetes _k8SClient;
    /// <summary>
    /// Environments access to the configuration of a pod
    /// </summary>
    private readonly IEnvironment _env;
    /// <summary>
    /// Logger instance
    /// </summary>
    private readonly ILogger _logger;

    /// <summary>
    /// Redis Interface client allowing to make calls to the Redis Cache
    /// </summary>
    private readonly RedisApiClient _redisClient;
    /// <summary>
    /// Name of the AWS registry used 
    /// </summary>
    private readonly string _awsRegistryName;

    public DeploymentService(IKubernetesBuilder kubernetesBuilder, IApiClientBuilder apiClientBuilder, IEnvironment env, ILogger<DeploymentService> logger)
    {
        _k8SClient = kubernetesBuilder.CreateKubernetesClient();
        _env = env;
        _logger = logger;
        _redisClient = apiClientBuilder.CreateRedisApiClient();
        _awsRegistryName = _env.GetEnvVariable("AWS_IMAGE_REGISTRY"); //TODO: replace with the parameters from the command line
    }

    /// <inheritdoc/>
    public async Task<bool> DeployAsync(TaskModel task)
    {
        var deployments = await _k8SClient.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
        var deploymentNames = deployments.Items.Select(d => d.Metadata.Name).ToArray();

        bool isSuccess = true;

        foreach (var seq in task.ActionSequence)
        {
            foreach (var service in seq.Services)
            {
                try
                {
                    var images = (await _redisClient.ContainerImageGetForInstanceAsync(service.Id))?.ToList();
                    if (images is null || images.Any() == false)
                    {
                        throw new IncorrectDataException("Images not defined for the Instance deployment");
                    }
                    // should only be just one image
                    foreach (var cim in images)
                    {
                        var deployedPair = await Deploy(cim);

                        //TODO: handle the check if the deployment or a service already exists
                        service.ServiceStatus = ServiceStatusEnum.Idle.GetStringValue();
                        service.ServiceInstanceId = Guid.Parse(deployedPair.Deployment.GetLabel("serviceId"));

                        //if (deployedPair.Service is not null && deployedPair.Service.Spec.ExternalIPs.Any())
                        //{
                        //    service.ServiceUrl = new Uri(deployedPair.Service.Spec.ExternalIPs[0]);
                        //}

                        //TODO save the specified actionPlan to the Redis

                    }
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "There was an error while deploying the service");
                    isSuccess = false;
                }
            }
        }

        return isSuccess;
    }
    /// <summary>
    /// Deploy the service based on the container information
    /// </summary>
    /// <param name="cim"></param>
    /// <returns></returns>
    public async Task<DeploymentPairModel> Deploy(ContainerImageModel cim)
    {
        var instanceId = Guid.NewGuid();

        var service = await Deploy<V1Service>(cim.K8SService, cim.Name, instanceId, nameof(cim.K8SService));

        var deployment = await Deploy<V1Deployment>(cim.K8SDeployment, cim.Name, instanceId, nameof(cim.K8SDeployment));

        return new DeploymentPairModel(deployment, service);
    }
    /// <summary>
    /// Deploy the Service of the specified type. Available types are <seealso cref="V1Service"/> and <seealso cref="V1Deployment"/>
    /// </summary>
    /// <typeparam name="T">Type of the service</typeparam>
    /// <param name="objectDefinition">string representation of the k8s yaml object</param>
    /// <param name="name">Name of the service to be deployed</param>
    /// <param name="instanceId"></param>
    /// <param name="propName"></param>
    /// <returns></returns>
    /// <exception cref="IncorrectDataException"></exception>
    /// <exception cref="UnableToParseYamlConfigException"></exception>
    public async Task<T> Deploy<T>(string objectDefinition, string name, Guid instanceId, string propName = null) where T : class
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

            obj = await _k8SClient.CreateNamespacedServiceAsync(srv, AppConfig.K8SNamespaceName) as T;
            return obj;
        }
        if (obj is V1Deployment depl)
        {
            depl.Metadata.SetServiceLabel(instanceId);

            obj = await _k8SClient.CreateNamespacedDeploymentAsync(depl, AppConfig.K8SNamespaceName) as T;
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
        if (name.Contains("redis"))
        {
            envList.Add(new V1EnvVar("REDIS_HOSTNAME", _env.GetEnvVariable("REDIS_HOSTNAME")));
            envList.Add(new V1EnvVar("REDIS_PORT", _env.GetEnvVariable("REDIS_PORT")));
        }

        var container = new V1Container()
        {
            Name = name,
            Image = K8SImageHelper.BuildImageName(_awsRegistryName, name, "latest"),
            ImagePullPolicy = AppConfig.AppConfiguration == "Release" && name == "redis-interface-api" ? "Always" : "Never",
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