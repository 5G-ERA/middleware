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

        /// <summary>
        /// Put all node topics into a single list.
        /// </summary>
        /// <param name="nodes"></param>
        /// <returns></returns>
        public List<RosTopicModel> GetAllNodeTopics(List<ROSNodeModel> nodes)
        {
            List<RosTopicModel> topics = new List<RosTopicModel>();

            foreach (ROSNodeModel node in nodes)
            {
                foreach (RosTopicModel pubTopic in node.Publications)
                {
                    if (!(topics.Contains(pubTopic)))
                    {
                        topics.Add(pubTopic);
                    }
                }
                foreach (RosTopicModel subTopic in node.Subscriptions)
                {
                    if (!(topics.Contains(subTopic)))
                    {
                        topics.Add(subTopic);
                    }
                }
            }
            return topics;
        }

    }
}
