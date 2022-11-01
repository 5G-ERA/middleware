using Middleware.Common.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Enums
{
    public class ROSNodeModel
    {
        public string name { get; set; }

        public List<RosTopicModel> Publications { get; set; }

        public List<RosTopicModel> Subscriptions { get; set; }

        public List<ROSServiceModel> Services { get; set; }

    }
}
