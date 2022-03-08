using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common
{
    public interface ILocatable
    {
        public Guid Id { get; set; }

        public Type RelationType { get; set; }
    }
}
