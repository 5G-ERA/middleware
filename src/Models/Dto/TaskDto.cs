using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "task-idx", StorageType = StorageType.Json, Prefixes = new[] { TaskDto.Prefix })]
public class TaskDto : Dto
{
    private const string Prefix = "Task";
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;

    [Indexed]
    public string Name { get; init; } = default!;

    [Indexed(Sortable = true)]
    public int TaskPriority { get; init; }

    /// <summary>
    /// Is the result of the task always the same
    /// </summary>
    [Indexed(Sortable = true)]
    public bool DeterministicTask { get; init; }

    [Indexed]
    public List<string>? Tags { get; init; }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new TaskModel()
        {
            Id = Guid.Parse(dto.Id),
            Name = dto.Name,
            TaskPriority = dto.TaskPriority,
            DeterministicTask = dto.DeterministicTask,
            Tags = dto.Tags
        };
    }
}