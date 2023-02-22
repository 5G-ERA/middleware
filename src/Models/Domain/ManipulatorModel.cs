using Middleware.Models.Dto.Ros;
using System.Xml.Linq;

namespace Middleware.Models.Domain
{
    public class ManipulatorModel
    {
        public string ActuatorName { get; set; }
        public int Dof { get; set; } //degree's of freedomn

        public int Number { get; set; }


        public Manipulator ToDto()
        {
            return new Manipulator()//
            {
                ActuatorName = ActuatorName,
                Dof = Dof,
                Number = Number
            };
        }
    }
}