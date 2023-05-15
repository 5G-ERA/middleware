using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain
{
    public class SensorModel
    {
        public string Name { get; set; } = default!;

        public string Type { get; set; } = default!;
        
        public string? Description { get; set; }
        /// <summary>
        /// List of the Node names published by the sensor
        /// </summary>
        public List<string> Nodes { get; set; } = new();

        public int Number { get; init; }

        public Sensor ToDto()
        {
            var domain = this;
            return new Sensor()
            {
                Name = domain.Name,
                Type = domain.Type,
                Description = domain.Description,
                Nodes = domain.Nodes,
                Number = domain.Number
            };
        }
    }
}