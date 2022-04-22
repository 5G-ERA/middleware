﻿using k8s.Models;
using Middleware.Common.Enums;
using Middleware.Common.Models;

namespace Middleware.Orchestrator.Deployment;

public interface IDeploymentService
{
    /// <summary>
    /// Deploy the services for the specified task plan
    /// </summary>
    /// <param name="task">Task plan with the defined action sequence and needed resources</param>
    /// <returns></returns>
    Task<bool> DeployAsync(TaskModel task);
    /// <summary>
    /// Creates startup deployment needed to instantiate the middleware.
    /// </summary>
    /// <param name="name">Name of the Middleware component to be deployed</param>
    /// <returns>Deployment for the middleware component. Created deployment is not instantiated</returns>
    V1Deployment CreateStartupDeployment(string name);
    /// <summary>
    /// Creates the service of the specified type with the metadata
    /// </summary>
    /// <param name="serviceImageName">Name of the service to be deployed</param>
    /// <param name="kind">Kind of the service to be deployed</param>
    /// <param name="meta">Metadata for the service from the existing deployment</param>
    /// <returns>Service of the specified type. Service has not been deployed</returns>
    V1Service CreateService(string serviceImageName, K8SServiceKindEnum kind, V1ObjectMeta meta);
}