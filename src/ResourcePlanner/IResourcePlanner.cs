using Middleware.Common.Models;
using System.Linq;


namespace Middleware.ResourcePlanner
{
    public interface IResourcePlanner
    {
        Task<TaskModel> Plan(TaskModel task, RobotModel robot);
        Task<TaskModel> RePlan(TaskModel currentTask, TaskModel oldTask, RobotModel robot, bool isFullReplan);
    }
}
