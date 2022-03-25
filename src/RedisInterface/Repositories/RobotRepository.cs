﻿using Middleware.Common.Models;
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
        public RobotRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(RedisDbIndexEnum.Robots, redisClient, redisGraph)
        {
        }

        public async Task<List<RelationModel>> GetRelation(string relationName)
        {
            List<RelationModel> relationModels = new List<RelationModel>();
            relationName = relationName?.ToUpper();
            ResultSet resultSet = await RedisGraph.Query("RESOURCE_PLANNER",
                "MATCH (x:ROBOT {ID:'ROBOT_1'}) MATCH (y) WHERE (x)-[: " + relationName + "]->(y) RETURN x,y");

            // BB: 24.03.2022
            // We are using the loop with 2 nested loops to retrieve the values from the graph
            // The values are structured in the following way:
            // First result contains the information about the objects that the relation initiates from
            // Secend results contains the information about the objects that the relation is pointing to
            // This structure will be universal for the explanation of all the queries on the redis graph
            for (int i = 0; i < resultSet.Results.Count; i++)
            {
                var res = resultSet.Results.ElementAt(i);                
                if (i % 2 == 0)
                {
                    foreach (RedisGraphResult node in res.Value)
                    {
                        var relationModel = new RelationModel();
                        relationModel.RelationName = relationName;
                        if (node is Node nd)
                        {
                            SetGraphModelValues(relationModel.InitiatesFrom, nd);
                        }
                        relationModels.Add(relationModel);
                    }
                }
                else
                {
                    foreach (RedisGraphResult node in res.Value)
                    {
                        var idxTmp = res.Value.IndexOf(node);
                        var relationModel = relationModels[idxTmp];
                        if (node is Node nd)
                        {
                            SetGraphModelValues(relationModel.PointsTo, nd);
                        }
                    }
                }
            }
            return relationModels;

        }

        public async Task<RobotModel> PatchRobotAsync(Guid id, RobotModel patch) 
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            RobotModel currentModel = JsonSerializer.Deserialize<RobotModel>(model);
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
