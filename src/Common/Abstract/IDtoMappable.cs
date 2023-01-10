using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Abstract
{
    public interface IDtoMappable<TDto>
    {
        TDto MapToDto();
    }
}
