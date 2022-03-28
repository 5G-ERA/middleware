using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using Middleware.RedisInterface.Repositories.Abstract;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using System.Text.Json;

namespace Middleware.RedisInterface.Repositories
{
    public class ActionRepository : BaseRepository<ActionModel>, IActionRepository
    {
        public ActionRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(RedisDbIndexEnum.Actions, redisClient, redisGraph)
        {
        }


        public async Task<ActionModel> PatchActionAsync(Guid id, ActionModel patch)
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            ActionModel currentModel = JsonSerializer.Deserialize<ActionModel>(model);
            if (!string.IsNullOrEmpty(patch.Order.ToString()))
            {
                currentModel.Order = patch.Order;
            }
            if (!string.IsNullOrEmpty(patch.Placement))
            {
                currentModel.Placement = patch.Placement;
            }
            if (!string.IsNullOrEmpty(patch.ActionPriority))
            {
                currentModel.ActionPriority = patch.ActionPriority;
            }
            if (!string.IsNullOrEmpty(patch.Services.ToString()))
            {
                currentModel.Services = patch.Services;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }


        public async Task<List<RelationModel>> GetRelation(Guid id, string relationName) 
        {
            List<RelationModel> relationModels = new List<RelationModel>();
            relationName = relationName?.ToUpper();
            ResultSet resultSet = await RedisGraph.Query("RESOURCE_PLANNER",
                "MATCH (x:ACTION {ID:'ACTION_1'}) MATCH (y) WHERE (x)-[: " + relationName + "]->(y) RETURN x,y");
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
