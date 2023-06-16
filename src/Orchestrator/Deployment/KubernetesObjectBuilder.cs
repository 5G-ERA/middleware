﻿using k8s;
using k8s.Models;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Exceptions;

namespace Middleware.Orchestrator.Deployment;

internal class KubernetesObjectBuilder : IKubernetesObjectBuilder
{
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
    public V1Service SerializeAndConfigureService(string service, string name, Guid serviceInstanceId)
    {
        if (string.IsNullOrWhiteSpace(service))
            throw new ArgumentException("Service definition cannot be empty.", nameof(service));

        var sanitized = service.SanitizeAsK8SYaml();
        var obj = KubernetesYaml.Deserialize<V1Service>(sanitized);

        if (obj is null) throw new UnableToParseYamlConfigException(name, nameof(ContainerImageModel.K8SService));

        obj.Validate();
        obj.Metadata.SetServiceLabel(serviceInstanceId);
        obj.Metadata.Name = name;
        throw new NotImplementedException();
    }

    /// <inheritdoc />
    public V1Service CreateDefaultService(string deploymentName, Guid serviceInstanceId)
    {
        throw new NotImplementedException();
    }

    public V1Service CreateStartupService(string serviceImageName, K8SServiceKindEnum kind, V1ObjectMeta meta)
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
    public V1Deployment SerializeAndConfigureDeployment(string deployment, Guid serviceInstanceId)
    {
        throw new NotImplementedException();
    }
}