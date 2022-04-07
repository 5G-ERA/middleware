using k8s;
using k8s.Models;
using Middleware.Common;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.RedisInterface;
using TaskModel = Middleware.Common.Models.TaskModel;

namespace Middleware.Orchestrator.Deployment;

public class DeploymentService : IDeploymentService
{
    /// <summary>
    /// Kubernetes client connection builder
    /// </summary>
    private readonly IKubernetesBuilder _k8SBuilder;
    /// <summary>
    /// Environments access to the configuration of a pod
    /// </summary>
    private readonly IEnvironment _env;
    /// <summary>
    /// Redis Interface client allowing to make calls to the Redis Cache
    /// </summary>
    private readonly RedisApiClient _redisClient;
    /// <summary>
    /// Name of the AWS registry used 
    /// </summary>
    private readonly string _awsRegistryName;

    public DeploymentService(IKubernetesBuilder k8SBuilder, IApiClientBuilder apiClientBuilder, IEnvironment env)
    {
        _k8SBuilder = k8SBuilder;
        _env = env;
        _redisClient = apiClientBuilder.CreateRedisApiClient();
        _awsRegistryName = _env.GetEnvVariable("AWS_IMAGE_REGISTRY"); //TODO: replace with the parameters from the command line
    }

    public async Task<bool> DeployAsync(TaskModel task)
    {
        var client = _k8SBuilder.CreateKubernetesClient();

        var deployments = await client.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
        var deploymentNames = deployments.Items.Select(d => d.Metadata.Name).ToArray();

        foreach (var seq in task.ActionSequence)
        {
            foreach (var service in seq.Services)
            {
                var v1Deployment = CreateStartupDeployment(service.ImageName);

                if (deploymentNames.Contains(v1Deployment.Metadata.Name))
                    continue;

                var result = await client.CreateNamespacedDeploymentAsync(v1Deployment, AppConfig.K8SNamespaceName);
            }
        }

        return false;
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