using Middleware.Models.Domain;

namespace Middleware.TaskPlanner.Services
{
    public interface IActionPlanner 
    {
        void Initialize(List<ActionModel> actionSequence, DateTime currentTime);

        Task<Tuple<TaskModel, RobotModel>> Plan(Guid taskId, Guid robotId);

        Task<Tuple<TaskModel, RobotModel>> InferActionSequence(Guid id, bool ContextKnown,  bool lockResource, List<DialogueModel> dialogueTemp, Guid robotId);

        Task<Tuple<TaskModel, TaskModel, RobotModel>> ReInferActionSequence(TaskModel oldTask, Guid RobotId, bool ContextKnown, bool CompleteReplan, List<DialogueModel> DialogueTemp);
    }

}



