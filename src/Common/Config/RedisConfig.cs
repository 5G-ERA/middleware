namespace Middleware.Common.Config
{
    public class RedisConfig
    {
        public const string ConfigName = "Redis";

        public string HostName { get; set; }

        public string Password { get; set; }
    }
}
