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
        public Guid Id { get; set; } = Guid.NewGuid();
        public DateTime CreationDate { get; set; } = DateTime.Now;

        public List<string> AvailableResources { get; } = new List<string>() { "CPU, GPU, RAM" };



    }
}


