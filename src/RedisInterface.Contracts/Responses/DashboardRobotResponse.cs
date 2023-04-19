using System.Text.Json.Serialization;

namespace Middleware.RedisInterface.Contracts.Responses
{
    public record DashboardRobotResponse
    {
        [JsonPropertyName("Id")]
        public Guid Id { get; set; }

        [JsonPropertyName("Name")]
        public string Name { get; set; }

        [JsonPropertyName("Status")]
        public string Status { get; set; }

        [JsonPropertyName("OnboardedTime")]
        public DateTime? OnboardedTime { get; set; }

        [JsonPropertyName("rosVersion")]
        public int RosVersion { get; set; }

        [JsonPropertyName("rosDistro")]
        public string RosDistro { get; set; }

        [JsonPropertyName("manufacturer")]
        public string Manufacturer { get; set; }

        public DashboardRobotResponse()
        {

        }
        public DashboardRobotResponse(Guid id, string name, string status, DateTime onboardedTime, int rosVersion,string rosDistro, string manufacturer)
        {
            Id = id;
            Name = name;
            Status = status;
            OnboardedTime = onboardedTime;
            RosVersion = rosVersion;
            RosDistro = rosDistro;
            Manufacturer = manufacturer;
        }

    }
}
