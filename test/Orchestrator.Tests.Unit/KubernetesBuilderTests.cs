using System;
using System.Collections.Generic;
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
    private readonly KubernetesBuilder _sut;
    private readonly IEnvironment _env = Substitute.For<IEnvironment>();
    private readonly ILogger<KubernetesBuilder> _logger = Substitute.For<ILogger<KubernetesBuilder>>();

    public KubernetesBuilderTests()
    {
        _sut = new KubernetesBuilder(_logger, _env);
    }

    [Fact]
    public void IsValidInClusterConfig_ShouldReturnTrue_WhenEnvVarsExistAndFilesArePresent()
    {
        // arrange
        _env.GetEnvVariable("KUBERNETES_SERVICE_HOST").Returns("192.168.0.1");
        _env.GetEnvVariable("KUBERNETES_SERVICE_PORT").Returns("443");
        _env.DirectoryExists(KubernetesBuilder.ServiceAccountPath).Returns(true);
        _env.GetFileNamesInDir(KubernetesBuilder.ServiceAccountPath)
            .Returns(new List<string>()
            {
                KubernetesBuilder.ServiceAccountNamespaceFileName,
                KubernetesBuilder.ServiceAccountRootCAKeyFileName,
                KubernetesBuilder.ServiceAccountTokenKeyFileName
            });
        // act
        var result = _sut.IsValidInClusterConfig();
        // assess
        result.Should().BeTrue();
    }

    [Fact]
    public void IsValidInClusterConfig_ShouldReturnFalse_WhenServiceAccountNotSet()
    {
        // arrange
        _env.GetEnvVariable("KUBERNETES_SERVICE_HOST").Returns("192.168.0.1");
        _env.GetEnvVariable("KUBERNETES_SERVICE_PORT").Returns("443");
        _env.DirectoryExists(KubernetesBuilder.ServiceAccountPath).Returns(false);
        _env.GetFileNamesInDir(KubernetesBuilder.ServiceAccountPath).Returns(new List<string>());
        // act
        var result = _sut.IsValidInClusterConfig();
        // assess
        result.Should().BeFalse();
    }

    [Fact]
    public void CreateKubernetesClient_ShouldThrowException_WhenEnvVarsAreNotSetAndKubeconfigDoesNotExist()
    {
        // arrange
        _env.GetEnvVariable("KUBERNETES_SERVICE_HOST").Returns("");
        _env.GetEnvVariable("KUBERNETES_SERVICE_PORT").Returns("");
        _env.FileExists(KubernetesBuilder.KubeConfigPath).Returns(false);
        // act
        Func<IKubernetes> act = () => _sut.CreateKubernetesClient();
        // assess
        act.Should().Throw<NotInK8SEnvironmentException>().WithMessage("The environment is not a Kubernetes environment, cannot instantiate the Middleware");
    }
}