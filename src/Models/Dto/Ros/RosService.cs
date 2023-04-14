using Middleware.Models.Domain;

namespace Middleware.Models.Dto.Ros;

public class RosService
{
    public string? Name { get; init; }
    public string? Description { get; init; }

    public ROSServiceModel ToModel()
    {
        return new ROSServiceModel()
        {
            Name = this.Name!,
            Description = this.Description!
        };
    }
}