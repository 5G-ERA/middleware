using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    internal class SliceModel
    {
        public Guid Id { get; set; }
        public string Name { get; set; }
        public bool IsDynamic { get; set; }
        public string Type { get; set; }
    }
}
