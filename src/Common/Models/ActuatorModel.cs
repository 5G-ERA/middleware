using Middleware.Common.Enums;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace Middleware.Common.Models
{
    public class ActuatorModel 
    {
        public string Name { get; set; }
        public string Type { get; set; }
        public int Dof { get; set; }
        public List<ROSNodeModel> Nodes { get; set; }

    }
}
