using Middleware.Models.Domain;

namespace Middleware.Models.Dto.Ros;

public class RosTopic 
{
    public string? Name { get; set; }
    public string? Type { get; set; }
    public string? Description { get; set; }
    public bool Enabled { get; set; }

    public RosTopicModel ToModel()
    {
        var dto = this;
        return new RosTopicModel()
        {
            Name = dto.Name,
            Type = dto.Type,
            Description = dto.Description,
            Enabled = dto.Enabled
        };
    }
}