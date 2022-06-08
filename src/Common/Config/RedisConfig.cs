using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Config
{
    public class RedisConfig
    {
        public const string ConfigName = "Redis";

        public string HostName { get; set; }

        public string User { get; set; }

        public string Password { get; set; }
    }
}
