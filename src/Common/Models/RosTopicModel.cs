using Middleware.Common.Enums;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    public class RosTopicModel
    {
        public string Name { get; set; }
        public string Type { get; set; }
        public string Description { get; set; }
        public bool Enabled { get; set; }

    }
 
}
