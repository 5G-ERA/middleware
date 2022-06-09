using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Config
{
    public class JwtConfig
    {
        public const string ConfigName = "JwtTokenConfig";

        public string Key { get; set; }
    }
}
