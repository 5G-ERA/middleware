using k8s;
using k8s.Models;
using Middleware.Common;
using Middleware.Common.ExtensionMethods;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Exceptions;
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
                    var images = (await _redisClient.ContainerImageGetForInstanceAsync(seq.Id))?.ToList();
                    if (images is null || images.Any() == false)
                    {
                        throw new IncorrectDataException("Images not defined for the Instance deployment");
                    }
                    // should only be just one image
                    foreach (var cim in images)
                    {
                        var deployment = await CreateAndDeployDeployment(cim);
                        if (deployment is null)
                        {
                            //TODO: sth
                        }

                        var deplService = await CreateAndDeployService(cim);

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

    private async Task<V1Service> CreateAndDeployService(ContainerImageModel cim)
    {
        if (string.IsNullOrWhiteSpace(cim.K8SDeployment))
        {
            _logger.LogInformation("Service definition for {Name} has not been specified, the instantiation of the service has been skipped", cim.Name);
            return null;
        }
        var sanitized = cim.K8SService.SanitizeAsK8SYaml();
        var tmpDeployment = Yaml.LoadFromString<V1Service>(sanitized);

        if (tmpDeployment == null)
        {
            throw new UnableToParseYamlConfigException(cim.Name, nameof(cim.K8SService));
        }

        var result = await _k8SClient.CreateNamespacedServiceAsync(tmpDeployment, AppConfig.K8SNamespaceName);
        return result;
    }

    private async Task<V1Deployment> CreateAndDeployDeployment(ContainerImageModel cim)
    {
        if (string.IsNullOrWhiteSpace(cim.K8SDeployment))
        {
            throw new IncorrectDataException($"Deployment for {cim.Name} not set");
        }
        var sanitized = cim.K8SDeployment.SanitizeAsK8SYaml();
        var tmpDeployment = Yaml.LoadFromString<V1Deployment>(sanitized);

        if (tmpDeployment == null)
        {
            throw new UnableToParseYamlConfigException(cim.Name, nameof(cim.K8SDeployment));
        }

        var result = await _k8SClient.CreateNamespacedDeploymentAsync(tmpDeployment, AppConfig.K8SNamespaceName);
        return result;
    }

    public V1Service CreateLoadBalancerService(string serviceImageName, V1ObjectMeta meta)
    {
        var spec = new V1ServiceSpec()
        {
            Ports = new List<V1ServicePort>() { new(80), new(443) },
            Selector = new Dictionary<string, string> { { "app", serviceImageName } }

        };

        var service = new V1Service()
        {
            Metadata = meta,
            ApiVersion = "api/v1",
            Kind = "LoadBalancer",
            Spec = spec
        };
        return service;
    }

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
            ImagePullPolicy = "Always",
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
            ApiVersion = "v1",
            Kind = "Deployment"
        };
        return dep;
    }
}