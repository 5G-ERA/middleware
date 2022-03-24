using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using Middleware.RedisInterface.Repositories.Abstract;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class RobotRepository : BaseRepository<RobotModel>, IRobotRepository
    {
        public RobotRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(RedisDbIndexEnum.Robots, redisClient, redisGraph)
        {
        }

        public async Task<List<RelationModel>> GetRelation()
        {
            List<RelationModel> relationModels = new List<RelationModel>();
            
            ResultSet resultSet = await RedisGraph.Query("RESOURCE_PLANNER", "MATCH (x: ROBOT{ID: 'ROBOT_1'})MATCH (y)WHERE (x)-[: CAN_REACH]->(y) RETURN x,y");
            foreach (var res in resultSet.Results)
            {
                foreach (RedisGraphResult val in res.Value)
                {
                    RobotModel model = new RobotModel();
                    if (val is ScalarResult<Guid> stringVal)
                        if (res.Key == "(x.id)")
                            model.Id = stringVal.Value;
                    //relationModels.Add(model);
                }
            }
            return relationModels;


            
        }
    }
}
