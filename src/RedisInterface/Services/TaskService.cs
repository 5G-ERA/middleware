using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Services;

internal class TaskService : ITaskService
{
    private readonly IActionRepository _actionRepository;
    private readonly IContainerImageRepository _containerImageRepository;
    private readonly IInstanceRepository _instanceRepository;
    private readonly ITaskRepository _taskRepository;

    public TaskService(ITaskRepository taskRepository, IActionRepository actionRepository,
        IInstanceRepository instanceRepository, IContainerImageRepository containerImageRepository)
    {
        _taskRepository = taskRepository;
        _actionRepository = actionRepository;
        _instanceRepository = instanceRepository;
        _containerImageRepository = containerImageRepository;
    }

    /// <inheritdoc />
    public async Task<TaskModel> GetFullTaskDefinitionAsync(Guid id)
    {
        var task = await _taskRepository.GetByIdAsync(id);

        if (task is null)
            return null;

        var actionRelations = await _taskRepository.GetRelation(id, "EXTENDS");

        var actions = new List<ActionModel>();
        foreach (var relation in actionRelations)
        {
            var action = await _actionRepository.GetByIdAsync(relation.PointsTo.Id);
            if (action is null)
                continue;

            actions.Add(action);
            var instanceRelations = await _actionRepository.GetRelation(action.Id, "NEEDS");
            foreach (var instanceRelation in instanceRelations)
            {
                var instance = await _instanceRepository.GetByIdAsync(instanceRelation.PointsTo.Id);
                if (instance is null)
                    continue;

                action.Services.Add(instance);


                var containers = await _containerImageRepository.GetImagesForInstanceAsync(instance.Id);
                if (!containers.Any())
                    continue;

                // should be always only one container
                instance.ContainerImage = containers.First();
            }
        }

        task.ActionSequence = actions;

        return task;
    }
}