using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain
{
    public class ROSServiceModel
    {
        public string Name { get; set; } = default!;
        public string? Description { get; set; }

        public RosService ToDto()
        {
            var domain = this;
            return new RosService()
            {
                Description = domain.Description,
                Name = domain.Name
            };
        }
    }
}