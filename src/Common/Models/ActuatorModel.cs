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
        public string ActuatorName { get; set; }
        public string ActuatorType { get; set; }
        public int number { get; set; }
        public List<ROSNodeModel> Nodes { get; set; }

    }
}
