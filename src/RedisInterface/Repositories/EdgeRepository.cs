﻿using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using Middleware.RedisInterface.Repositories.Abstract;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class EdgeRepository : BaseRepository<EdgeModel>, IEdgeRepository
    {
        public EdgeRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(RedisDbIndexEnum.Edges, redisClient, redisGraph)
        {
        }

        public async Task<List<RelationModel>> GetRelation(Guid id, string relationName) 
        {
            List<RelationModel> relationModels = new List<RelationModel>();
            relationName = relationName?.ToUpper();
            ResultSet resultSet = await RedisGraph.Query("RESOURCE_PLANNER",
                "MATCH (x:EDGE {ID:'EDGE_1'}) MATCH (y) WHERE (x)-[: " + relationName + "]->(y) RETURN x,y");

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
    }
}
