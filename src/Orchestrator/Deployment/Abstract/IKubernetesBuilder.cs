using k8s;

namespace Middleware.Orchestrator.Deployment
{
    public interface IKubernetesBuilder
    {
        /// <summary>
        /// Creates the appropriate <see cref="Kubernetes"/> client
        /// </summary>
        /// <returns></returns>
        IKubernetes CreateKubernetesClient();
        /// <summary>
        /// Check if the Current system matches the in-cluster configuration required
        /// </summary>
        /// <returns></returns>
        bool IsValidInClusterConfig();
    }
}
