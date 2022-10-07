using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    public class RobotManipulatorModel
    {
        public string ActuatorName { get; set; }
        public int dof { get; set; } //degree's of freedomn

        public int number { get; set; }
    }
}
