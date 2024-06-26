﻿using JetBrains.Annotations;
using k8s.Models;
using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Contracts;
using Middleware.Orchestrator.Models;

namespace Middleware.Orchestrator.Deployment;

public interface IKubernetesObjectBuilder
{
    /// <summary>
    ///     Creates startup deployment for specified middleware component and specified version
    /// </summary>
    /// <param name="name">Middleware service name</param>
    /// <param name="tag">Middleware version</param>
    /// <returns></returns>
    V1Deployment CreateStartupDeployment(string name, string tag);

    /// <summary>
    ///     Deserializes string to the deployment definition that will be used to deploy NetApp
    /// </summary>
    /// <param name="deployment"></param>
    /// <param name="serviceInstanceId"></param>
    /// <param name="name"></param>
    /// <param name="thisLocation"></param>
    /// <param name="shouldUseSimTime"></param>
    /// <returns></returns>
    V1Deployment DeserializeAndConfigureDeployment(string deployment, Guid serviceInstanceId, string name,
        ILocation thisLocation, bool shouldUseSimTime);

    /// <summary>
    /// Enable data persistence for nhe NetApp using AWS S3
    /// </summary>
    /// <param name="dpl"></param>
    /// <param name="config"></param>
    /// <param name="netAppDataKey"></param>
    /// <returns></returns>
    V1Deployment EnableDataPersistence(V1Deployment dpl, SystemConfigModel config, string netAppDataKey);
    /// <summary>
    ///     Deserializes string to the service definition that will be used to expose the NetApp internally or externally
    /// </summary>
    /// <param name="service"></param>
    /// <param name="name"></param>
    /// <param name="serviceInstanceId"></param>
    /// <returns></returns>
    V1Service DeserializeAndConfigureService(string service, string name, Guid serviceInstanceId);

    /// <summary>
    ///     Creates the default service definition to expose the service internally using default http and https ports
    /// </summary>
    /// <param name="deploymentName"></param>
    /// <param name="serviceInstanceId"></param>
    /// <param name="depl"></param>
    /// <returns></returns>
    V1Service CreateDefaultService(string deploymentName, Guid serviceInstanceId, V1Deployment depl);

    /// <summary>
    ///     Creates startup service to expose the deployment within a cluster
    /// </summary>
    /// <param name="serviceImageName">Name of the service</param>
    /// <param name="kind">Kind of the Kubernetes Service</param>
    /// <param name="meta">Metadata of the deployment to be exposed</param>
    /// <param name="nodePort"></param>
    /// <returns></returns>
    V1Service CreateStartupService(string serviceImageName, K8SServiceKind kind, V1ObjectMeta meta, int? nodePort = null);

    /// <summary>
    ///     Configures cross NetApp communication by providing the addresses via environment variables
    /// </summary>
    /// <param name="netApps">The list of NetApp configurations</param>
    void ConfigureCrossNetAppConnection(IReadOnlyList<DeploymentPair> netApps);

    /// <summary>
    ///     Creates the deployment configuration for the Inter Relay NetApp responsible for routing traffic to multiple NetApps
    /// </summary>
    /// <param name="actionPlanId"></param>
    /// <param name="action"></param>
    /// <param name="pairs"></param>
    /// <param name="cfg"></param>
    /// <returns></returns>
    DeploymentPair CreateInterRelayNetAppDeploymentConfig(Guid actionPlanId, [NotNull] ActionModel action,
        [NotNull] IReadOnlyList<DeploymentPair> pairs, SystemConfigModel cfg);

    /// <summary>
    ///     Creates the labels used to identify the Inter Relay NetApp
    /// </summary>
    /// <param name="actionPlanId"></param>
    /// <param name="actionId"></param>
    /// <returns></returns>
    Dictionary<string, string> CreateInterRelayNetAppLabels(Guid actionPlanId, Guid actionId);

    V1Deployment AddLinkerdAnnotation(V1Deployment deployment);
}