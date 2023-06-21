using k8s;
using k8s.Models;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Exceptions;
using Middleware.Orchestrator.Models;

namespace Middleware.Orchestrator.Deployment;

internal class KubernetesObjectBuilder : IKubernetesObjectBuilder
{
    /// <summary>
    ///     Defines an interval in which the NetApps report heartbeat to the Middleware
    /// </summary>
    private const int ReportIntervalInSeconds = 5;

    private readonly IConfiguration _config;

    /// <summary>
    ///     Name of the container registry used
    /// </summary>
    private readonly string _containerRegistryName;

    private readonly IEnvironment _env;

    public KubernetesObjectBuilder(IEnvironment env, IConfiguration config)
    {
        _env = env;
        _config = config;
        _containerRegistryName = _env.GetEnvVariable("IMAGE_REGISTRY") ?? "ghcr.io/5g-era";
    }

    /// <inheritdoc />
    public V1Service DeserializeAndConfigureService(string service, string name, Guid serviceInstanceId)
    {
        if (string.IsNullOrWhiteSpace(service))
            return null;

        var sanitized = service.SanitizeAsK8SYaml();
        var obj = KubernetesYaml.Deserialize<V1Service>(sanitized);

        if (obj is null) throw new UnableToParseYamlConfigException(name, nameof(ContainerImageModel.K8SService));

        obj.Validate();
        obj.Metadata.SetServiceLabel(serviceInstanceId);
        obj.Metadata.Name = name;
        throw new NotImplementedException();
    }

    public V1Service CreateStartupService(string serviceImageName, K8SServiceKind kind, V1ObjectMeta meta)
    {
        var spec = new V1ServiceSpec
        {
            Ports = CreateDefaultHttpPorts(),
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
    public void ConfigureCrossNetAppConnection(IReadOnlyList<DeploymentPair> netApps)
    {
        // get environment variable names 
        // evaluate the addresses
        // attach environment variable to remaining deployments
        var dict = netApps
            .Select(n => new { NetAppName = n.Name, ServiceName = n.Service.Metadata.Name })
            .ToList();

        foreach (var netApp in netApps)
        {
            foreach (var kvp in dict)
            {
                if (kvp.NetAppName == netApp.Name)
                    continue;
                var container = netApp.Deployment.Spec.Template.Spec.Containers.First();

                // what about multiple ports exposed by a service?
                var envVarName = $"{kvp.NetAppName.ToUpper()}";
                var envVarValue =
                    $"{kvp.ServiceName.Replace('-', '_').ToUpper()}.{AppConfig.K8SNamespaceName}.svc.cluster.local";

                container.Env.Add(new(envVarName, envVarValue));
            }
        }
    }

    public V1Deployment CreateStartupDeployment(string name, string tag)
    {
        var mwConfig = _config.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
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

    /// <inheritdoc />
    public V1Deployment DeserializeAndConfigureDeployment(string deploymentStr, Guid serviceInstanceId, string name)
    {
        if (string.IsNullOrWhiteSpace(deploymentStr))
            throw new ArgumentException("Service definition cannot be empty.", nameof(deploymentStr));

        var sanitized = deploymentStr.SanitizeAsK8SYaml();
        var obj = KubernetesYaml.Deserialize<V1Deployment>(sanitized);

        if (obj is null) throw new UnableToParseYamlConfigException(name, nameof(ContainerImageModel.K8SService));

        obj.Metadata.SetServiceLabel(serviceInstanceId);
        obj.Metadata.Name = name;
        foreach (var container in obj.Spec.Template.Spec.Containers)
        {
            var envVars = container.Env is not null
                ? new(container.Env)
                : new List<V1EnvVar>();

            envVars.Add(new("NETAPP_ID", serviceInstanceId.ToString()));
            envVars.Add(new("MIDDLEWARE_ADDRESS", AppConfig.GetMiddlewareAddress()));
            envVars.Add(new("MIDDLEWARE_REPORT_INTERVAL", ReportIntervalInSeconds.ToString()));

            container.Env = envVars;
        }

        return obj;
    }

    /// <inheritdoc />
    public V1Service CreateDefaultService(string deploymentName, Guid serviceInstanceId, V1Deployment depl)
    {
        var ports = depl.Spec.Template.Spec.Containers.SelectMany(p =>
            p.Ports.Select(pp => new CommonPort(pp.Name, pp.ContainerPort, pp.Protocol))).ToList();


        var servicePorts = ports.Any()
            ? MapToServicePorts(ports)
            : CreateDefaultHttpPorts();


        var spec = new V1ServiceSpec
        {
            Ports = servicePorts,
            Selector = new Dictionary<string, string> { { "app", deploymentName } },
            Type = K8SServiceKind.ClusterIp.GetStringValue()
        };

        var service = new V1Service
        {
            Metadata = new()
            {
                Name = depl.Metadata.Name,
                Labels = depl.Metadata.Labels
            },
            ApiVersion = "v1",
            Spec = spec,
            Kind = "Service"
        };
        return service;
    }

    private static List<V1ServicePort> MapToServicePorts(IEnumerable<CommonPort> ports)
    {
        return ports.Select(p => new V1ServicePort(p.Port, p.Protocol, p.Name)).ToList();
    }

    private static List<V1ServicePort> CreateDefaultHttpPorts()
    {
        return new()
        {
            new(80, "TCP", "http", null, "TCP", 80),
            new(443, "TCP", "https", null, "TCP", 80)
        };
    }
}

internal record CommonPort(string Name, int Port, string Protocol);