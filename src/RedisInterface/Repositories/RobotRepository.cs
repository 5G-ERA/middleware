using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using Middleware.RedisInterface.Repositories.Abstract;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using System.Text.Json;

namespace Middleware.RedisInterface.Repositories
{
    public class RobotRepository : BaseRepository<RobotModel>, IRobotRepository
    {
        public RobotRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<RobotRepository> logger) : base(RedisDbIndexEnum.Robot, redisClient, redisGraph, logger)
        {
        }

        public async Task<RobotModel> PatchRobotAsync(Guid id, RobotModel patch) 
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            RobotModel currentModel = JsonSerializer.Deserialize<RobotModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.RobotName))
            {
                currentModel.RobotName = patch.RobotName;
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
            if (!string.IsNullOrEmpty(patch.TaskList.ToString()))
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
            if (!string.IsNullOrEmpty(patch.Sensors.ToString()))
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
            if (!string.IsNullOrEmpty(patch.Questions.ToString()))
            {
                currentModel.Questions = patch.Questions;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }
    }
}
