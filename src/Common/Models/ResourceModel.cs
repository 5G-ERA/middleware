using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    [Obsolete] //obsolete will exec
    public class ResourceModel
    {
        public Guid TaskId { get; set; } = Guid.NewGuid();

        public string TaskPriority { get; set; } = "HIGH/MEDIUM/LOW";

        public Guid ActionPlanId { get; set; } = Guid.NewGuid();
        public DateTime CreationDate { get; set; } = DateTime.Now;

        public List<string> AvailableResources { get; } = new List<string>() { "CPU, GPU, RAM" };
    }

}
