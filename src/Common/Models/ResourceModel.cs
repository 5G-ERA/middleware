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

        {

  "ActionSequence": [
   {
      "ActionId": 2,
      "Order": 0,
      "ActionPriority": "1/2/3",
      "Placement": "EDGE/CLOUD",
      "ServiceId/Image name": "Object detection service"
   } 
  ]
}
    }

}
