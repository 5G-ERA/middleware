using Middleware.Models.Domain.Ros;

namespace Middleware.Models.Dto.Ros;

public class RosAction
{
    public string Name { get; set; } = default!;
    public string Type { get; set; } = default!;

    public RosActionModel ToModel()
    {
        var dto = this;
        return new()
        {
            Name = dto.Name,
            Type = dto.Type
        };
    }
}