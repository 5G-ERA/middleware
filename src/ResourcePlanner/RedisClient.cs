using Middleware.Common.Models;

namespace Middleware.ResourcePlanner
{
    public class RedisClient
    {

        public async Task<List<InstanceModel>> GetServicesForAction()
        {
            return new List<InstanceModel>() { new InstanceModel() { ImageName = "SLAM", Id = Guid.NewGuid() },
                                                new InstanceModel() { ImageName = "Object detection", Id = Guid.NewGuid()}};
        }
    }
}
