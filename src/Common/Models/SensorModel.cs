using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    public class SensorModel
    {
        public string SensorName { get; set; }
        public string SensorType { get; set; }
        public string SensorLocation  { get; set; }
        public string SensorDescription { get; set; }
        public List<RosTopicModel> RosTopicPub { get; set; } //A sensor can publish multiple topics
    }
}
