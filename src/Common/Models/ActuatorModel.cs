using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Middleware.Common.Enums;


namespace Middleware.Common.Models
{
    public class ActuatorModel
    {
        public string Name { get; set; }
        public string Type { get; set; }

        public int Number { get; set; }

        public List<string> Nodes { get; set; }

    }
}
