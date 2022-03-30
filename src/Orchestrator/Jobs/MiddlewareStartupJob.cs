using k8s;
using k8s.Models;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Exceptions;
using Quartz;

namespace Middleware.Orchestrator.Jobs
{
    public class MiddlewareStartupJob : BaseJob<MiddlewareStartupJob>
    {
        private const string K8SNamespaceName = "5G-ERA";

        private readonly IKubernetesBuilder _kubeBuilder;

        public MiddlewareStartupJob(ILogger<MiddlewareStartupJob> logger, IKubernetesBuilder kubeBuilder) : base(logger)
        {
            _kubeBuilder = kubeBuilder;
        }

        protected override async Task ExecuteJobAsync(IJobExecutionContext context)
        {
            Logger.LogInformation("Executed");
            try
            {
                var client = _kubeBuilder.CreateKubernetesClient();

                await CheckAndCreateNamespace(client);

            }
            catch (NotInK8sEnvironmentException ex)
            {
                Logger.LogWarning("Could not instantiate the Kubernetes Client", ex);
                throw;
            }
        }

        /// <summary>
        /// Checks if the namespace for the Middleware exists, if not, it creates it
        /// </summary>
        /// <param name="kubeClient"></param>
        /// <returns></returns>
        private async Task CheckAndCreateNamespace(IKubernetes kubeClient)
        {
            var namespaces = await kubeClient.ListNamespaceAsync();

            var exists = namespaces.Items.Any(n => n.Name() == K8SNamespaceName);

            if (exists)
                return;

            var ns = new V1Namespace()
            {
                Metadata = new V1ObjectMeta()
                {
                    Name = "five-g-era"
                }
            };

            var result = await kubeClient.CreateNamespaceAsync(ns);
            Logger.LogDebug(result.ToString());

        }
    }
}
