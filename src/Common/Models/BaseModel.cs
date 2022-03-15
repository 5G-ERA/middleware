using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    public abstract class BaseModel
    {
        public abstract Guid Id { get; set; }
    }
}
