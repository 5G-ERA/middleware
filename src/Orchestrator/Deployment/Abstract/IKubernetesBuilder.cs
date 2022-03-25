using k8s;

namespace Middleware.Orchestrator.Deployment
{
    public interface IKubernetesBuilder
    {
        IKubernetes CreateKubernetesClient();
    }
}