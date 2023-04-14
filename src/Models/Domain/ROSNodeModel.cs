﻿using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain
{
    public class ROSNodeModel
    {
        public string Name { get; set; }

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

        public RosNode ToDto()
        {
            var domain = this;
            return new RosNode()
            {
                Name = domain.Name,
                Publications = domain.Publications?.Select(x => x.ToDto()).ToList()!,
                Subscriptions = domain.Subscriptions?.Select(x => x.ToDto()).ToList()!,
                Services = domain.Services?.Select(x => x.ToDto()).ToList()!
            };
        }
    }
}