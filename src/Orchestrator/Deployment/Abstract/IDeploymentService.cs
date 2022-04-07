using k8s.Models;
using Middleware.Common.Models;

namespace Middleware.Orchestrator.Deployment;

public interface IDeploymentService
{
    Task<bool> DeployAsync(TaskModel task);
    V1Deployment CreateStartupDeployment(string name);
    V1Service CreateLoadBalancerService(string serviceImageName, V1ObjectMeta meta);
}