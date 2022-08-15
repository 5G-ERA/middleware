using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    internal class SliceModel
    {
        public Guid SliceId { get; set; }
        public string SliceName { get; set; }
        public bool DynamimcSlcie { get; set; }
        public  string TypeSlice { get; set; }
    }
}
