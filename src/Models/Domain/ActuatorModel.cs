using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain
{
    public class ActuatorModel
    {
        public string Name { get; set; }
        public string Type { get; set; }

        public int Number { get; set; }

        public List<string> Nodes { get; set; }

        public Actuator ToDto()
        {
            return new Actuator()//
            {
                Name = Name,
                Type = Type,
                Number = Number,
                Nodes = Nodes

            };
        }
    }
}