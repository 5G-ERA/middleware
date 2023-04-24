using Middleware.Models.Domain;

namespace Middleware.ResourcePlanner.Models
{
    public record ResourceInput(TaskModel Task, RobotModel Robot, bool FullReplan);
    
}
