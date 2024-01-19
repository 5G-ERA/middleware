using Middleware.Models.Domain.Ros;

namespace Middleware.Models.Dto.Ros;

public class RosService
{
    public string? Name { get; init; }
    public string? Description { get; init; }

    public RosServiceModel ToModel()
    {
        return new()
        {
            Name = Name!,
            Description = Description!
        };
    }
}