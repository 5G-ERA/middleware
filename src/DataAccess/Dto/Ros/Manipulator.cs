using Redis.OM.Modeling;

namespace Middleware.DataAccess.Dto.Ros
{
    internal class Manipulator
    {
        /// <summary>
        /// Actuator Name
        /// </summary>
        [Indexed]
        public string? ActuatorName { get; set; }
        /// <summary>
        /// Degrees of freedom
        /// </summary>
        [Indexed]
        public int Dof { get; set; } 
    }
}
