using Middleware.Common.Models;

namespace Middleware.TaskPlanner.Services
{
    public interface IActionPlanner 
    {
        void Initialize(List<ActionModel> actionSequence, DateTime currentTime);

        Task<Tuple<TaskModel, RobotModel>> InferActionSequence(Guid id, bool lockResource, List<DialogueModel> dialogueTemp, Guid robotId);
    }

}



