using Middleware.Common.Models;

namespace Middleware.ResourcePlanner
{
    public class RedisClient
    {

        public async Task<List<InstanceModel>> GetServicesForAction()
        {
            return new List<InstanceModel>() { new InstanceModel() { Name = "SLAM", Id = Guid.NewGuid() },
                                                new InstanceModel() { Name = "Object detection", Id = Guid.NewGuid()}};
        }
    }
}
