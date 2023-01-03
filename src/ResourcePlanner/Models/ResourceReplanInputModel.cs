using Middleware.Common.Models;

namespace Middleware.ResourcePlanner.Models
{
    public record ResourceReplanInputModel(TaskModel Task, TaskModel oldTask, RobotModel Robot, bool FullReplan);

}
