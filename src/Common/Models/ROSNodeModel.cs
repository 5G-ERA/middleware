using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace Middleware.Common.Models
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
        public HashSet<RosTopicModel> GetAllNodeTopics()
        {
            HashSet<RosTopicModel> topics = new HashSet<RosTopicModel>();

            foreach (RosTopicModel pubTopic in Publications)
            {
                topics.Add(pubTopic);
            }
            foreach (RosTopicModel subTopic in Subscriptions)
            {
                topics.Add(subTopic);
            }

            return topics;
        }

    }
}
