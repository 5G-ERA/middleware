using Middleware.Common.Repositories;
using Middleware.Common.Repositories.Abstract;

namespace Middleware.RedisInterface.Services
{
    public class DashboardService : IDashboardService
    {
        private readonly IRobotRepository _robotRepository;            
        private readonly ITaskRepository _taskRepository;
        public DashboardService(IRobotRepository robotRepository, ITaskRepository taskRepository)
        {
            _taskRepository = taskRepository;
            _robotRepository = robotRepository;
        }

        public async Task GetRobotStatusList(Guid id)
        {
            var robot = await _robotRepository.GetByIdAsync(id);
            // DO X
            //return Task.FromResult(0);
        }

        public Task GetRobotStatusList()
        {
            throw new NotImplementedException();
        }
    }
}
