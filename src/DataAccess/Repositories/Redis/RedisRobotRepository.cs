using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Neo4j.Driver;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories;

public class RedisRobotRepository : RedisRepository<RobotModel, RobotDto>, IRobotRepository
{
    /// <summary>
    ///     Default constructor
    /// </summary>
    /// <param name="redisClient"></param>
    /// <param name="redisGraph"></param>
    /// <param name="logger"></param>
    public RedisRobotRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, Microsoft.Extensions.Logging.ILogger<RedisRobotRepository> logger, IDriver driver) : base(
        provider, redisGraph, false, logger, driver)
    {
    }

    /// <summary>
    ///     Patching properties for RobotModel
    /// </summary>
    /// <param name="id"></param>
    /// <param name="patch"></param>
    /// <returns> Patched model </returns>
    public async Task<RobotModel> PatchRobotAsync(Guid id, RobotModel patch)
    {
        var currentModel = await GetByIdAsync(id);

        if (currentModel == null) return null;
        if (!string.IsNullOrEmpty(patch.Name)) currentModel.Name = patch.Name;
        if (!string.IsNullOrEmpty(patch.RosVersion.ToString())) currentModel.RosVersion = patch.RosVersion;
        if (!string.IsNullOrEmpty(patch.RosDistro)) currentModel.RosDistro = patch.RosDistro;
        if (!string.IsNullOrEmpty(patch.MaximumPayload.ToString())) currentModel.MaximumPayload = patch.MaximumPayload;
        if (!string.IsNullOrEmpty(patch.MaximumTranslationalVelocity.ToString()))
            currentModel.MaximumTranslationalVelocity = patch.MaximumTranslationalVelocity;
        if (!string.IsNullOrEmpty(patch.MaximumRotationalVelocity.ToString()))
            currentModel.MaximumRotationalVelocity = patch.MaximumRotationalVelocity;
        if (!string.IsNullOrEmpty(patch.RobotWeight.ToString())) currentModel.RobotWeight = patch.RobotWeight;
        if (patch.ROSRepo != null && Uri.IsWellFormedUriString(patch.ROSRepo.ToString(), UriKind.RelativeOrAbsolute))
            currentModel.ROSRepo = patch.ROSRepo;
        if (patch.ROSNodes != null) currentModel.ROSNodes = patch.ROSNodes;
        if (!string.IsNullOrEmpty(patch.Manufacturer)) currentModel.Manufacturer = patch.Manufacturer;
        if (patch.ManufacturerUrl != null &&
            Uri.IsWellFormedUriString(patch.ManufacturerUrl.ToString(), UriKind.RelativeOrAbsolute))
            currentModel.ManufacturerUrl = patch.ManufacturerUrl;
        if (!string.IsNullOrEmpty(patch.RobotModelName)) currentModel.RobotModelName = patch.RobotModelName;
        if (!string.IsNullOrEmpty(patch.RobotStatus)) currentModel.RobotStatus = patch.RobotStatus;
        if (patch.TaskList != null) currentModel.TaskList = patch.TaskList;
        if (!string.IsNullOrEmpty(patch.BatteryStatus.ToString())) currentModel.BatteryStatus = patch.BatteryStatus;
        if (!string.IsNullOrEmpty(patch.MacAddress)) currentModel.MacAddress = patch.MacAddress;
        if (!string.IsNullOrEmpty(patch.LocomotionSystem)) currentModel.LocomotionSystem = patch.LocomotionSystem;
        if (patch.Sensors != null) currentModel.Sensors = patch.Sensors;
        if (patch.Actuators != null) currentModel.Actuators = patch.Actuators;
        if (patch.Manipulators != null) currentModel.Manipulators = patch.Manipulators;
        if (!string.IsNullOrEmpty(patch.Cpu.ToString())) currentModel.Cpu = patch.Cpu;
        if (!string.IsNullOrEmpty(patch.Ram.ToString())) currentModel.Ram = patch.Ram;
        if (!string.IsNullOrEmpty(patch.StorageDisk.ToString())) currentModel.StorageDisk = patch.StorageDisk;
        if (!string.IsNullOrEmpty(patch.NumberCores.ToString())) currentModel.NumberCores = patch.NumberCores;
        if (patch.Questions != null) currentModel.Questions = patch.Questions;
        if (!string.IsNullOrEmpty(patch.LastUpdatedTime.ToString()))
            currentModel.LastUpdatedTime = patch.LastUpdatedTime;
        if (!string.IsNullOrEmpty(patch.OnboardedTime.ToString())) currentModel.OnboardedTime = patch.OnboardedTime;
        await UpdateAsync(currentModel);
        return currentModel;
    }

    /// <summary>
    ///     Get all the edges models that have conection to the robot. TODO - it should be bi-directional.
    /// </summary>
    /// <param name="robotId"></param>
    /// <returns></returns>
    public async Task<List<EdgeModel>> GetConnectedEdgesIdsAsync(Guid robotId)
    {
        var edges = new List<EdgeModel>();
        var robotRelations = await GetRelation(robotId, "CAN_REACH");
        foreach (var relationModel in robotRelations)
        {
            if (relationModel.PointsTo.Type == "EDGE")
            {
                var edgeObject = new EdgeModel();
                edgeObject.Id = relationModel.PointsTo.Id;
                edgeObject.Name = relationModel.PointsTo.Name;
                edges.Add(edgeObject);
            }
        }

        return edges;
    }

    /// <summary>
    ///     Get all the clouds models that have conection to the robot. TODO - it should be bi-directional.
    /// </summary>
    /// <param name="robotId"></param>
    /// <returns>list of cloudsModels</returns>
    public async Task<List<CloudModel>> GetConnectedCloudsIdsAsync(Guid robotId)
    {
        var clouds = new List<CloudModel>();
        var robotRelations = await GetRelation(robotId, "CAN_REACH");
        foreach (var relationModel in robotRelations)
        {
            if (relationModel.PointsTo.Type == "CLOUD")
            {
                var cloudObject = new CloudModel();
                cloudObject.Id = relationModel.PointsTo.Id;
                cloudObject.Name = relationModel.PointsTo.Name;
                clouds.Add(cloudObject);
            }
        }

        return clouds;
    }
}