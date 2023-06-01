using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain
{
    public class RosTopicModel
    {
        public string Name { get; set; } = default!;
        public string? Type { get; set; }
        public string? Description { get; set; }
        public bool Enabled { get; set; }

        /// <summary>
        /// Set the topic to be enabled.
        /// </summary>
        public void Enable()
        {
            Enabled = true;
        }

        /// <summary>
        /// Set the topic to be disabled.
        /// </summary>
        public void Disable()
        {
            Enabled = false;
        }

        public RosTopic ToDto()
        {
            return new RosTopic
            {
                Name = Name,
                Type = Type,
                Description = Description,
                Enabled = Enabled
                
            };
        }
    }
}