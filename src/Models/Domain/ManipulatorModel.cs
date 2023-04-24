using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain
{
    public class ManipulatorModel
    {
        public string ActuatorName { get; set; }
        /// <summary>
        /// Degree's of freedom
        /// </summary>
        public int Dof { get; set; }
        public int Number { get; set; }
        public string Type { get; set; }


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