using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "task-idx", StorageType = StorageType.Json, Prefixes = new[] { TaskDto.Prefix })]
public class TaskDto : Dto
{
    public const string Prefix = "Task";
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; }

    [Indexed]
    public string? Name { get; set; }
   
    
    [Indexed(Sortable = true)]
    public int TaskPriority { get; set; }
   
    
    [Indexed(Sortable = true)]
    public bool DeterministicTask { get; set; } // The result is always the same if true.
    
    /*[Indexed]
    public List<ActionDto> ActionSequence { get; set; }*/
    [Indexed]
    public List<string> Tags { get; set; }
 

    public override BaseModel ToModel()
    {
        var dto = this;
        return new TaskModel()
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name,
            TaskPriority = dto.TaskPriority,
            DeterministicTask = dto.DeterministicTask,
            Tags = dto.Tags
        };
    }
}