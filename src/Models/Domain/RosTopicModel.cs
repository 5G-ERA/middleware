using Middleware.Models.Dto.Hardware;
using Middleware.Models.Dto;
using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain
{
    public class RosTopicModel
    {
        public string Name { get; set; }
        public string Type { get; set; }
        public string Description { get; set; }
        public bool Enabled { get; set; }

        /// <summary>
        /// Set the topic to be enabled.
        /// </summary>
        /// <param name="topic"></param>
        public void Enable()
        {
            Enabled = true;
        }

        /// <summary>
        /// Set the topic to be disabled.
        /// </summary>
        /// <param name="topic"></param>
        public void Disable()
        {
            Enabled = false;
        }

    }
}