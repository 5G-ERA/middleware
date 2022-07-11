using Middleware.Common.Models;
using Middleware.Common.Enums;
using Middleware.Common.Repositories.Abstract;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using System.Text.Json;
using Microsoft.Extensions.Logging;

namespace Middleware.Common.Repositories
{
    public class RobotRepository : BaseRepository<RobotModel>, IRobotRepository
    {
        

        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public RobotRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<RobotRepository> logger) : base(RedisDbIndexEnum.Robot, redisClient, redisGraph, logger, true)
        {
        }

        /// <summary>
        /// Patching properties for RobotModel
        /// </summary>
        /// <param name="id"></param>
        /// <param name="patch"></param>
        /// <returns> Patched model </returns>
        public async Task<RobotModel> PatchRobotAsync(Guid id, RobotModel patch) 
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            RobotModel currentModel = JsonSerializer.Deserialize<RobotModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }
            if (!string.IsNullOrEmpty(patch.Manufacturer))
            {
                currentModel.Manufacturer = patch.Manufacturer;
            }
            if (!string.IsNullOrEmpty(patch.RobotModelName))
            {
                currentModel.RobotModelName = patch.RobotModelName;
            }
            if (!string.IsNullOrEmpty(patch.RobotStatus))
            {
                currentModel.RobotStatus = patch.RobotStatus;
            }
            if (patch.TaskList != null)
            {
                currentModel.TaskList = patch.TaskList; 
            }
            if (!string.IsNullOrEmpty(patch.BatteryStatus.ToString()))
            {
                currentModel.BatteryStatus = patch.BatteryStatus;
            }
            if (!string.IsNullOrEmpty(patch.MacAddress))
            {
                currentModel.MacAddress = patch.MacAddress;
            }
            if (!string.IsNullOrEmpty(patch.LocomotionSystem))
            {
                currentModel.LocomotionSystem = patch.LocomotionSystem;
            }
            if (patch.Sensors != null)
            {
                currentModel.Sensors = patch.Sensors;
            }
            if (!string.IsNullOrEmpty(patch.Cpu.ToString()))
            {
                currentModel.Cpu = patch.Cpu;
            }
            if (!string.IsNullOrEmpty(patch.Ram.ToString()))
            {
                currentModel.Ram = patch.Ram;
            }
            if (!string.IsNullOrEmpty(patch.VirtualRam.ToString()))
            {
                currentModel.VirtualRam = patch.VirtualRam;
            }
            if (!string.IsNullOrEmpty(patch.StorageDisk.ToString()))
            {
                currentModel.StorageDisk = patch.StorageDisk;
            }
            if (!string.IsNullOrEmpty(patch.NumberCores.ToString()))
            {
                currentModel.NumberCores = patch.NumberCores;
            }
            if (patch.Questions != null)
            {
                currentModel.Questions = patch.Questions;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }

        public async Task<List<Guid>> GetConnectedEdgesIdsAsync(Guid robotId) 
        {
            List<Guid> edgeIds = new List<Guid>();
            List<RelationModel> robotRelations = await GetRelation(robotId, "CAN_REACH");
            foreach (RelationModel relationModel in robotRelations)
            {
                if (relationModel.PointsTo.Type == "EDGE") 
                {
                    edgeIds.Add(relationModel.PointsTo.Id);
                }
            }
            return edgeIds;
        } 

        public async Task<List<Guid>> GetFreeEdgesIdsAsync (List<Guid> listofEdgesConnectedtoRobot)
        {
            List<Guid> freeEdges = new List<Guid>();
            foreach (Guid edges in listofEdgesConnectedtoRobot)
            {
                // BB 11.07.2022: update on this, let's not create the new function, it doesn't make any sense to do so
                // I have added teh parameter to the GetRelation method mased on the parameter on which we will modify the query
                // Example call of the GetRelation() method in line 128
                // [Any] --[relation]--> [Edge with id]
                List<RelationModel> robotRelations = await GetRelation(edges, "LOCATED_AT", RelationDirection.Incoming);
                foreach (RelationModel relationModel in robotRelations) 
                {
                    if (relationModel.PointsTo != null)
                    {
                        freeEdges.Add(relationModel.PointsTo.Id);
                    }
                }
            }
            return freeEdges;

          

        }
    }
}
