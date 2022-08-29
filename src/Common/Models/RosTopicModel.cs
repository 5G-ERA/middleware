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
        public string Family { get; set; } //Sensor data topic, odometry topic, 
        public string Description { get; set; }
    }
}
