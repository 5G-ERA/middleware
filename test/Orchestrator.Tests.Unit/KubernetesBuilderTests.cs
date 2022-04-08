using System;
using FluentAssertions;
using k8s;
using Microsoft.Extensions.Logging;
using Middleware.Common;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Exceptions;
using NSubstitute;
using Xunit;

namespace Orchestrator.Tests.Unit;

public class KubernetesBuilderTests
{
    private readonly IKubernetesBuilder _sut;
    private readonly IEnvironment _env = Substitute.For<IEnvironment>();
    private readonly ILogger<KubernetesBuilder> _logger = Substitute.For<ILogger<KubernetesBuilder>>();

    public KubernetesBuilderTests()
    {
        _sut = new KubernetesBuilder(_logger, _env);
    }

    [Fact]
    public void CreateKubernetesClient_ShouldCreateClient_WhenEnvironmentVarsAreSet()
    {
        // arrange
        _env.GetEnvVariable("KUBERNETES_SERVICE_HOST").Returns("192.168.0.1");
        _env.GetEnvVariable("KUBERNETES_SERVICE_PORT").Returns("443");
        // act
        var client = _sut.CreateKubernetesClient();
        // assess
        client.Should().NotBeNull();
    }
    [Fact]
    public void CreateKubernetesClient_ShouldThrowException_WhenEnvironmentVarsAreNotSet()
    {
        // arrange
        _env.GetEnvVariable("KUBERNETES_SERVICE_HOST").Returns("");
        _env.GetEnvVariable("KUBERNETES_SERVICE_PORT").Returns("");
        // act
        Func<IKubernetes> act = () => _sut.CreateKubernetesClient();
        // assess
        act.Should().Throw<NotInK8SEnvironmentException>().WithMessage("The environment is not a Kubernetes environment, cannot instantiate the Middleware");
    }
}