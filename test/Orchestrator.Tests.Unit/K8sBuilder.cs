using System.Collections.Generic;
using k8s.Models;

namespace Orchestrator.Tests.Unit;

internal static class K8SBuilder
{
    public static V1Deployment CreateExampleDeployment()
    {
        return new()
        {
            ApiVersion = "apps/v1",
            Kind = "Deployment",
            Metadata = new()
            {
                Name = "example",
                Labels = new Dictionary<string, string>
                {
                    { "app", "example" }
                }
            },
            Spec = new()
            {
                Template = new()
                {
                    Metadata = new()
                    {
                        Name = "example",
                        Labels = new Dictionary<string, string>
                        {
                            { "app", "example" }
                        }
                    },
                    Spec = new()
                    {
                        Containers = new List<V1Container>
                        {
                            new()
                            {
                                Name = "example",
                                Image = "redis",
                                Ports = new List<V1ContainerPort>
                                {
                                    new(6379)
                                }
                            }
                        }
                    }
                }
            }
        };
    }
}