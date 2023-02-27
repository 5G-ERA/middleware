using System.Text.Json.Serialization;

namespace Middleware.RedisInterface.Contracts.Responses
{
    public record NetAppsDetailsResponse
    {
        [JsonPropertyName("name")]
        public string Name { get; set; }

        [JsonPropertyName("family")]
        public string Family { get; set; }

        [JsonPropertyName("rosVersion")]
        public int RosVersion { get; set; }

        [JsonPropertyName("rosDistro")]
        public string RosDistro { get; set; }

        [JsonPropertyName("onboardedTime")]
        public DateTime? OnboardedTime { get; set; }

        public NetAppsDetailsResponse()
        {
        }
        public NetAppsDetailsResponse(string name, string family, int rosVersion, string rosDistro , DateTime onboardedTime)
        {
            Name = name;
            Family = family;
            RosVersion = rosVersion;
            RosDistro = rosDistro;
            OnboardedTime = onboardedTime;

        }
    }
}
