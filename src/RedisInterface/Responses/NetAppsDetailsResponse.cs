using System.Text.Json.Serialization;

namespace Middleware.RedisInterface.Responses
{
    public record NetAppsDetailsResponse
    {
        [JsonPropertyName("NetAppName")]
        public string NetAppName { get; set; }

        [JsonPropertyName("NetAppFamily")]
        public string NetAppFamily { get; set; }

        [JsonPropertyName("ROSVersion")]
        public int ROSVersion { get; set; }

        [JsonPropertyName("ROSDistro")]
        public string ROSDistro { get; set; }

        [JsonPropertyName("OnboardedTime")]
        public DateTime? OnboardedTime { get; set; }

        public NetAppsDetailsResponse()
        {
        }
        public NetAppsDetailsResponse(string netAppName, string netAppFamily, int rosVersion, string rosDistro , DateTime onboardedTime)
        {
            NetAppName = netAppName;
            NetAppFamily = netAppFamily;
            ROSVersion = rosVersion;
            ROSDistro = rosDistro;
            OnboardedTime = onboardedTime;

        }
    }
}
