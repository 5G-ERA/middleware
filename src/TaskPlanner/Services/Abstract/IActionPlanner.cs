using Middleware.Common.Models;

namespace Middleware.TaskPlanner.Services
{
    public interface IActionPlanner 
    {
        void Initialize(List<ActionModel> actionSequence, DateTime currentTime);

        Task<Tuple<TaskModel, RobotModel>> InferActionSequence(Guid id, string description, bool ContextKnown,  bool lockResource, List<DialogueModel> dialogueTemp, Guid robotId);

        Task<Tuple<TaskModel, TaskModel, RobotModel>> ReInferActionSequence(TaskModel oldTask, string description, Guid RobotId, bool ContextKnown, bool CompleteReplan, List<DialogueModel> DialogueTemp);
    }

}



