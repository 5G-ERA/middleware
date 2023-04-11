using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain
{
    public class RosTopicModel
    {
        public string Name { get; set; } = default!;
        public string Type { get; set; } = default!;
        public string? Description { get; set; } = default!;
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
        public RosTopic ToDto()
        {
            return new RosTopic()//
            {
                Name = Name,
                Type = Type,
                Description = Description,
                Enabled = Enabled
                
            };
        }
    }
}