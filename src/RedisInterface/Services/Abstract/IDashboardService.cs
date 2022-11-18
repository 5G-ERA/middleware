using Middleware.RedisInterface.Responses;

namespace Middleware.RedisInterface.Services
{
    public interface IDashboardService
    {
        Task<List<TaskRobotResponse>> GetRobotStatusListAsync();
    }
}
