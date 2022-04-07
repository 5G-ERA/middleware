﻿using k8s;
using k8s.Models;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Exceptions;
using Quartz;

namespace Middleware.Orchestrator.Jobs
{
    public class MiddlewareStartupJob : BaseJob<MiddlewareStartupJob>
    {
        private readonly IKubernetesBuilder _kubeBuilder;
        private readonly IDeploymentService _deploymentService;

        public MiddlewareStartupJob(ILogger<MiddlewareStartupJob> logger, IKubernetesBuilder kubeBuilder, IDeploymentService deploymentService) : base(logger)
        {
            _kubeBuilder = kubeBuilder;
            _deploymentService = deploymentService;
        }

        protected override async Task ExecuteJobAsync(IJobExecutionContext context)
        {
            Logger.LogInformation("Executed");
            try
            {
                var client = _kubeBuilder.CreateKubernetesClient();

                await InstantiateMiddleware(client);

            }
            catch (NotInK8sEnvironmentException ex)
            {
                Logger.LogWarning("Could not instantiate the Kubernetes Client", ex);
                throw;
            }
        }

        /// <summary>
        /// Instantiates the pods for the middleware
        /// </summary>
        /// <param name="kubeClient"></param>
        /// <returns></returns>
        private async Task InstantiateMiddleware(IKubernetes kubeClient)
        {
            var namespaces = await kubeClient.ListNamespaceAsync();

            var exists = namespaces.Items.Any(n => n.Name() == AppConfig.K8SNamespaceName);

            if (exists == false)
                return;

            try
            {
                var deployments = await kubeClient.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
                var deploymentNames = deployments.Items.Select(d => d.Metadata.Name).ToArray();

                var images = new List<string>
                    {"gateway", "redis-interface-api", "resource-planner-api", "task-planner-api"};

                foreach (string service in images)
                {
                    var v1Deployment = _deploymentService.CreateStartupDeployment(service);

                    if (deploymentNames.Contains(v1Deployment.Metadata.Name))
                        continue;

                    var result = await kubeClient.CreateNamespacedDeploymentAsync(v1Deployment, AppConfig.K8SNamespaceName);

                    if (service == "gateway")
                    {
                        var lbService = _deploymentService.CreateLoadBalancerService(service, result.Metadata);
                        var createdService = await kubeClient.CreateNamespacedServiceAsync(lbService, AppConfig.K8SNamespaceName);
                    }
                }
            }
            catch (k8s.Autorest.HttpOperationException httpOperationException)
            {
                var phase = httpOperationException.Response.ReasonPhrase;
                var content = httpOperationException.Response.Content;
                Logger.LogError(httpOperationException, "Unable to deploy the resource to k8s:{phase}, {content}", phase, content);
            }
        }
    }
}
