using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    public class GraphEntityModel
    {
        public Guid Id { get; set; }
        public string Type { get; set; }

        public string Name { get; set; }
    }
}
