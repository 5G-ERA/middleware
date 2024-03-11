using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading.Tasks;
using k8s;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Publishers;
using Middleware.RedisInterface.Sdk;
using NSubstitute;
using Xunit;

namespace Orchestrator.Tests.Unit.Deployment;

public class DeploymentServiceTests
{
    private readonly IKubernetes _kube = Substitute.For<IKubernetes>();
    private readonly IKubernetesBuilder _kubeBuilder = Substitute.For<IKubernetesBuilder>();
    private readonly IKubernetesObjectBuilder _kubernetesObjectBuilder = Substitute.For<IKubernetesObjectBuilder>();
    private readonly ILogger<DeploymentService> _logger = Substitute.For<ILogger<DeploymentService>>();

    private readonly IOptions<MiddlewareConfig> _mwConfig = Options.Create(new MiddlewareConfig
    {
        InstanceName = "test-MW",
        InstanceType = "Edge",
        Organization = "5G-ERA-TST"
    });

    private readonly IPublishingService _publishingService = Substitute.For<IPublishingService>();
    private readonly IRedisInterfaceClient _redisInterface = Substitute.For<IRedisInterfaceClient>();
    private readonly IRosConnectionBuilderFactory _rosConnection = Substitute.For<IRosConnectionBuilderFactory>();
    private readonly ISystemConfigRepository _settingsRepo = Substitute.For<ISystemConfigRepository>();
    private readonly IKubernetesWrapper _kubeWrapper = Substitute.For<IKubernetesWrapper>();

    private readonly DeploymentService _sut;

    public DeploymentServiceTests()
    {
        _kubeBuilder.CreateKubernetesClient().Returns(_kube);
        _sut = new(_logger, _redisInterface, _mwConfig, _kubernetesObjectBuilder,
            _rosConnection, _publishingService, _settingsRepo, _kubeWrapper);
    }

    /* Also to be tested for deployment:
     *  - data persistence
     *  - inter-relay deployment
     *  - setting netapp address
     *  - name change if netapp with specified name is already deployed
     *  - error handling
     * 
     */
    [Fact(Skip = "Test not ready due to the complexity of the used functionalities")]
    public async Task DeployPlanAsync_ShouldCreateAndDeploySimpleActionPlan_WhenContainerImagesAreConfiguredCorrectly()
    {
        //arrange
        var actionPlan = GetActionPlan();
        /*to collect:
         * robot definition
         *  system config
         * current location definition
         * netappDataKey
         * currently running deployment names
         */
        
        /* in function runtime:
         * 
         *  - retrieve container image relation to current task
         *  - netapp deployment 
         *  - publishing netapp entry messages
         *  - adding relation
         *  - save actionSequence to redis
         *  - inter-realy deployment
         */
        
        var instanceId = actionPlan.ActionSequence!.First().Services.First().ServiceInstanceId;
        
        // act
        
        //act
        await _sut.DeployActionPlanAsync(actionPlan, /*TODO: update robot id*/ Guid.Empty);
        //assert
        /*
         * validate functions called
         * validate the result
         * check logs
         * 
         */
        
        /* Additional improvements:
         *  - cache read files
         *  - add IDisposable to the test class
         * 
         */
    }

    private TaskModel? GetActionPlan()
    {
        string filePath = "files/actionPlan.json";

        // Read the JSON file
        string json = File.ReadAllText(filePath);

        // Parse JSON content into a Person object
        return JsonSerializer.Deserialize<TaskModel>(json);
    }
}