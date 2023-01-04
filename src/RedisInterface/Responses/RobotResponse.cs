using System.Text.Json.Serialization;

namespace Middleware.RedisInterface.Responses
{
    public record RobotResponse
    {
        [JsonPropertyName("RobotId")]
        public Guid RobotId { get; set; }

        [JsonPropertyName("RobotName")]
        public string RobotName { get; set; }

        [JsonPropertyName("Status")]
        public string Status { get; set; }

        [JsonPropertyName("OnboardedTime")]
        public DateTime OnboardedTime { get; set; }

        [JsonPropertyName("ROSVersion")]
        public int ROSVersion { get; set; }

        [JsonPropertyName("ROSDistro")]
        public string ROSDistro { get; set; }

        [JsonPropertyName("Manufacturer")]
        public string Manufacturer { get; set; }

        public RobotResponse()
        {

        }
        public RobotResponse(Guid robotId, string robotName, string status, DateTime onboardedTime, int rosVersion,string rosDistro, string manufacturer)
        {
            RobotId = robotId;
            RobotName = robotName;
            Status = status;
            OnboardedTime = onboardedTime;
            ROSVersion = rosVersion;
            ROSDistro = rosDistro;
            Manufacturer = manufacturer;
        }

    }
}
