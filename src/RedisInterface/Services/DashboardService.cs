using Middleware.Common.Repositories;
using Middleware.Common.Repositories.Abstract;
using Middleware.RedisInterface.Responses;

namespace Middleware.RedisInterface.Services
{
    public class DashboardService : IDashboardService
    {
        private readonly IRobotRepository _robotRepository;            
        private readonly ITaskRepository _taskRepository;
        private readonly IActionPlanRepository _actionPlanRepository;

        public DashboardService(IRobotRepository robotRepository, ITaskRepository taskRepository, IActionPlanRepository actionPlanRepository)
        {
            _actionPlanRepository = actionPlanRepository;
            _taskRepository = taskRepository;
            _robotRepository = robotRepository;
        }

        public async Task<List<TaskRobotResponse>> GetRobotStatusListAsync()
        {
            var robots = await _robotRepository.GetAllAsync();
            var actionPlans = await _actionPlanRepository.GetAllAsync();
            var tasks = await _taskRepository.GetAllAsync();

            var responses = new List<TaskRobotResponse>();
            foreach (var ap in actionPlans)
            {
                var task = tasks.Where(t => t.Id == ap.TaskId).FirstOrDefault();
                var robot = robots.Where(r => r.Id == ap.RobotId).FirstOrDefault();
                if (task is null || robot is null)
                    continue;

                var response = new TaskRobotResponse(
                    ap.RobotId,
                    robot.Name,
                    task.Id,
                    task.Name,
                    ap.TaskStartedAt,
                    ap.Status == "completed" ? ap.LastStatusChange : null,
                    robot.RobotStatus);
                
                responses.Add(response);
            }

            return responses;
        }
    }
}
