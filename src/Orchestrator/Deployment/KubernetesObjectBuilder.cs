﻿using System.Text.Json;
using System.Text.Json.Serialization;
using JetBrains.Annotations;
using k8s;
using k8s.Models;
using Microsoft.Extensions.Options;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Contracts;
using Middleware.Models.ExtensionMethods;
using Middleware.Orchestrator.Exceptions;
using Middleware.Orchestrator.Models;

namespace Middleware.Orchestrator.Deployment;

internal class KubernetesObjectBuilder : IKubernetesObjectBuilder
{
    /// <summary>
    ///     Defines an interval in which the NetApps report heartbeat to the Middleware
    /// </summary>
    private const int ReportIntervalInSeconds = 5;
    
    private readonly IOptions<MiddlewareConfig> _mwConfig;

    /// <summary>
    ///     Name of the container registry used
    /// </summary>
    private readonly string _containerRegistryName;

    private readonly IEnvironment _env;
    
    public KubernetesObjectBuilder(IEnvironment env, IOptions<MiddlewareConfig> mwConfig)
    {
        _env = env;
        _mwConfig = mwConfig;
        _containerRegistryName = _env.GetEnvVariable("IMAGE_REGISTRY")?.TrimEnd('/') ?? "ghcr.io/5g-era";
    }

    /// <inheritdoc />
    public V1Deployment EnableDataPersistence(V1Deployment dpl, SystemConfigModel config, string netAppDataKey)
    {
        var volume = new V1Volume()
        {
            Name = "shared",
            EmptyDir = new()
        };
        if (dpl.Spec.Template.Spec.Volumes is null)
            dpl.Spec.Template.Spec.Volumes = new List<V1Volume>();
        
        dpl.Spec.Template.Spec.Volumes.Add(volume);
        
        var hermesFetch = CreateHermesFetchContainer(netAppDataKey, config);
        if (dpl.Spec.Template.Spec.InitContainers is null)
            dpl.Spec.Template.Spec.InitContainers = new List<V1Container>();
        
        dpl.Spec.Template.Spec.InitContainers.Add(hermesFetch);
        
        //update existing container
        var netApp = dpl.Spec.Template.Spec.Containers.First();
        netApp.Env.Add(new("NETAPP_DATA_DIR", "/data/" + netAppDataKey));
        netApp.Env.Add(new("NETAPP_KEY", netAppDataKey));
        netApp.VolumeMounts = new List<V1VolumeMount>
        {
            new()
            {
                Name = "shared",
                MountPath = "/data"
            }
        };
        var hermesPost = CreateHermesPostContainer(netAppDataKey, config);
        dpl.Spec.Template.Spec.Containers.Add(hermesPost);
        
        return dpl;
    }

    private V1Container CreateHermesPostContainer(string netAppKey, SystemConfigModel config)
    {
        const string dir = "/data/upload";
        var hermes = CreateHermesContainer(netAppKey, config);
        hermes.Name += "-post";
        hermes.Env.Add(new("POST_DIR",dir));
        hermes.Args.Add("post");
        hermes.VolumeMounts = new List<V1VolumeMount>
        {
            new()
            {
                Name = "shared",
                MountPath = dir
            }
        };
        return hermes;
    }
    private V1Container CreateHermesFetchContainer(string netAppKey, SystemConfigModel config)
    {
        const string dir = "/data/download";
        var hermes = CreateHermesContainer(netAppKey, config);
        hermes.Name += "-fetch";
        hermes.Env.Add(new("FETCH_DIR",dir));
        hermes.Args.Add("fetch");
        hermes.VolumeMounts = new List<V1VolumeMount>
        {
            new()
            {
                Name = "shared",
                MountPath = dir
            }
        };
        return hermes;
    }
    private V1Container CreateHermesContainer(string netAppDataKey, SystemConfigModel config)
    {
        var hermes = new V1Container
        {
            Name = "hermes",
            Image = config.HermesContainer,
            Env = new List<V1EnvVar>
            {
                new("AWS_ACCESS_KEY_ID", _env.GetEnvVariable("AWS_ACCESS_KEY_ID")),
                new("AWS_SECRET_ACCESS_KEY", _env.GetEnvVariable("AWS_SECRET_ACCESS_KEY")),
                new("AWS_REGION", config.S3DataPersistenceRegion),
                new("AWS_BUCKET", config.S3DataPersistenceBucketName),
                new("NETAPP_KEY", netAppDataKey)
            },
            Args = new List<string>()
        };
        return hermes;
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

        return obj;
    }

    public V1Service CreateStartupService(string serviceImageName, K8SServiceKind kind, V1ObjectMeta meta,
        int? nodePort = null)
    {
        var spec = new V1ServiceSpec
        {
            Ports = CreateDefaultHttpPorts(nodePort),
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
            new("Middleware__Address", _mwConfig.Value.Address),
            new("Middleware__Organization", _mwConfig.Value.Organization),
            new("Middleware__InstanceName", _mwConfig.Value.InstanceName),
            new("Middleware__InstanceType", _mwConfig.Value.InstanceType),
            new("CustomLogger__LoggerName", _env.GetEnvVariable("CustomLogger__LoggerName")),
            new("CustomLogger__Url", _env.GetEnvVariable("CustomLogger__Url")),
            new("CustomLogger__User", _env.GetEnvVariable("CustomLogger__User")),
            new("CustomLogger__Password", _env.GetEnvVariable("CustomLogger__Password")),
            new("Slice__Hostname", _env.GetEnvVariable("Slice__Hostname")),
            new("RabbitMQ__Address", _env.GetEnvVariable("RabbitMQ__Address")),
            new("RabbitMQ__User", _env.GetEnvVariable("RabbitMQ__User")),
            new("RabbitMQ__Pass", _env.GetEnvVariable("RabbitMQ__Pass")),
            new("RabbitMQ__Port", _env.GetEnvVariable("RabbitMQ__Port")),
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
            ImagePullPolicy = "Always",
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
    public V1Service CreateDefaultService(string deploymentName, Guid serviceInstanceId, V1Deployment depl)
    {
        var ports = depl.Spec.Template.Spec.Containers.SelectMany(p =>
            p.Ports?.Select(pp => new CommonPort(pp.Name, pp.ContainerPort, pp.Protocol)) ??
            Enumerable.Empty<CommonPort>()).ToList();

        var servicePorts = ports.Any()
            ? MapToServicePorts(ports)
            : CreateDefaultHttpPorts();


        var spec = new V1ServiceSpec
        {
            Ports = servicePorts,
            Selector = depl.Spec.Selector.MatchLabels,
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

    public Dictionary<string, string> CreateInterRelayNetAppLabels(Guid actionPlanId, Guid actionId)
    {
        return new()
        {
            { "actionPlanId", actionPlanId.ToString() },
            { "actionId", actionId.ToString() }
        };
    }

    /// <inheritdoc />
    public V1Deployment DeserializeAndConfigureDeployment(string deploymentStr, Guid serviceInstanceId, string name,
        ILocation thisLocation, bool shouldUseSimTime)
    {
        if (string.IsNullOrWhiteSpace(deploymentStr))
            throw new ArgumentException("Service definition cannot be empty.", nameof(deploymentStr));

        var sanitized = deploymentStr.SanitizeAsK8SYaml();
        var obj = KubernetesYaml.Deserialize<V1Deployment>(sanitized);

        if (obj is null) throw new UnableToParseYamlConfigException(name, nameof(ContainerImageModel.K8SDeployment));
        var keysToModify = new List<string>();
        foreach (var kvp in obj.Spec.Selector.MatchLabels)
        {
            if (kvp.Value == obj.Metadata.Name)
                keysToModify.Add(kvp.Key);
        }
        foreach (var key in keysToModify)
        {
            obj.Spec.Selector.MatchLabels[key] = name.SanitizeAsK8sObjectName();
        }

        obj.Spec.Template.Metadata.Labels = obj.Spec.Selector.MatchLabels;
        obj.Metadata.SetServiceLabel(serviceInstanceId);
        obj.Metadata.Name = name.SanitizeAsK8sObjectName();
        foreach (var container in obj.Spec.Template.Spec.Containers)
        {
            var envVars = container.Env is not null
                ? new(container.Env)
                : new List<V1EnvVar>();

            envVars.Add(new("NETAPP_ID", serviceInstanceId.ToString()));
            envVars.Add(new("NETAPP_NAME", name));
            envVars.Add(new("MIDDLEWARE_ADDRESS", thisLocation.GetNetAppStatusReportAddress()));
            envVars.Add(new("MIDDLEWARE_REPORT_INTERVAL", ReportIntervalInSeconds.ToString()));
            if (shouldUseSimTime)
            {
                envVars.Add(new("USE_SIM_TIME", "True"));
            }
            container.Env = envVars;
        }

        return obj;
    }

    /// <inheritdoc />
    public DeploymentPair CreateInterRelayNetAppDeploymentConfig(Guid actionPlanId, ActionModel action,
        IReadOnlyList<DeploymentPair> pairs, SystemConfigModel cfg)
    {
        if (action == null) throw new ArgumentNullException(nameof(action));
        if (pairs == null) throw new ArgumentNullException(nameof(pairs));
        if (actionPlanId == Guid.Empty) throw new ArgumentNullException(nameof(actionPlanId));

        var serviceInstanceId = action.Services.First().ServiceInstanceId;
        var configs = new List<MultiNetAppConfigRow>();
        foreach (var pair in pairs)
        {
            var topics = pair.Instance!.RosTopicsSub.Select(t => t.Name).ToList();
            var config = new MultiNetAppConfigRow
            {
                Address = $"http://{pair.Name}",
                Topics = topics,
                Services = new()
            };
            configs.Add(config);
        }

        var configString = JsonSerializer.Serialize(configs);

        var deployment = CreateInterRelayDeploymentDefinition(actionPlanId, action.Id, configString, cfg);

        var service = CreateDefaultService(deployment.Name(), serviceInstanceId, deployment);

        return new(deployment.Name(), deployment, service, serviceInstanceId);
    }

    public Dictionary<string, string> CreateInterRelayNetAppMatchLabels(string relayName)
    {
        return new()
        {
            { "app", relayName }
        };
    }

    private V1Deployment CreateInterRelayDeploymentDefinition(Guid actionPlanId, Guid actionId,
        string configString, SystemConfigModel cfg)
    {
        var relayName = "inter-relay-netapp".AddRandomSuffix();
        var matchLabels = CreateInterRelayNetAppMatchLabels(relayName);
        return new()
        {
            ApiVersion = "apps/v1",
            Metadata = new()
            {
                Name = relayName,
                Labels = CreateInterRelayNetAppLabels(actionPlanId, actionId)
            },
            Spec = new()
            {
                Selector = new()
                {
                    MatchLabels = matchLabels
                },
                Replicas = 1,
                Template = new()
                {
                    Metadata = new()
                    {
                        Name = relayName,
                        Labels = matchLabels
                    },
                    Spec = new()
                    {
                        Containers = new List<V1Container>
                        {
                            new()
                            {
                                Name = "relay",
                                Image = cfg.RosInterRelayNetAppContainer,
                                Env = new List<V1EnvVar>
                                {
                                    new("RELAYS_LIST", configString),
                                    new("NETAPP_PORT", "80")
                                }
                            }
                        }
                    }
                }
            }
        };
    }

    private static List<V1ServicePort> MapToServicePorts(IEnumerable<CommonPort> ports)
    {
        return ports.Select(p => new V1ServicePort(p.Port, p.Protocol, p.Name)).ToList();
    }

    private List<V1ServicePort> CreateDefaultHttpPorts(int? nodePort = null)
    {
        return new()
        {
            new(80, "TCP", "http", nodePort, "TCP", 80)
        };
    }
    
    public V1Deployment AddLinkerdAnnotation(V1Deployment deployment)
    {
        deployment.Spec.Template.SetAnnotation("linkerd.io/inject", "enabled");
        deployment.SetAnnotation("linkerd.io/inject", "enabled");
        return deployment;
    }

    /// <summary>
    ///     This class is used to represent the config for the MultiRelayNetApp
    /// </summary>
    private class MultiNetAppConfigRow
    {
        [JsonPropertyName("relay_address")]
        public string Address { [UsedImplicitly] get; set; }

        [JsonPropertyName("topics")]
        public List<string> Topics { [UsedImplicitly] get; set; }

        /// <summary>
        ///     Do not remove, will be used to support ROS services
        /// </summary>
        [JsonPropertyName("services")]
        public List<string> Services { [UsedImplicitly]get; set; } = new();
    }
}

internal record CommonPort(string Name, int Port, string Protocol);