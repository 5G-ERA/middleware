using Middleware.Common.Models;

namespace Middleware.ResourcePlanner.Models
{
    public record ResourceInput(TaskModel Task, RobotModel Robot, bool FullReplan);

}
