﻿using System.Collections.Generic;
using System.Text.Json;
using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Enums;
using NReJSON;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using ILogger = Serilog.ILogger;


namespace Middleware.DataAccess.Repositories
{
    public class RedisRobotRepository : RedisRepository<RobotModel, RobotDto>, IRobotRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>

        public RedisRobotRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(provider, redisGraph, true, logger)
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
            RobotModel? currentModel = await GetByIdAsync(id);

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
            await UpdateAsync(currentModel);
            return currentModel;
        }

        /// <summary>
        /// Get all the edges models that have conection to the robot. TODO - it should be bi-directional.
        /// </summary>
        /// <param name="robotId"></param>
        /// <returns></returns>
        public async Task<List<EdgeModel>> GetConnectedEdgesIdsAsync(Guid robotId)
        {
            List<EdgeModel> edges = new List<EdgeModel>();
            List<RelationModel> robotRelations = await GetRelation(robotId, "CAN_REACH");
            foreach (RelationModel relationModel in robotRelations)
            {
                if (relationModel.PointsTo.Type == "EDGE")
                {
                    EdgeModel edgeObject = new EdgeModel();
                    edgeObject.Id = relationModel.PointsTo.Id;
                    edgeObject.Name = relationModel.PointsTo.Name;
                    edges.Add(edgeObject);
                }
            }
            return edges;
        }

        /// <summary>
        /// Get all the clouds models that have conection to the robot. TODO - it should be bi-directional.
        /// </summary>
        /// <param name="robotId"></param>
        /// <returns>list of cloudsModels</returns>
        public async Task<List<CloudModel>> GetConnectedCloudsIdsAsync(Guid robotId)
        {
            List<CloudModel> clouds = new List<CloudModel>();
            List<RelationModel> robotRelations = await GetRelation(robotId, "CAN_REACH");
            foreach (RelationModel relationModel in robotRelations)
            {
                if (relationModel.PointsTo.Type == "CLOUD")
                {
                    CloudModel cloudObject = new CloudModel();
                    cloudObject.Id = relationModel.PointsTo.Id;
                    cloudObject.Name = relationModel.PointsTo.Name;
                    clouds.Add(cloudObject);
                }
            }
            return clouds;
        }


    }
}
