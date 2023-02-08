using System.Text.Json.Serialization;

namespace Middleware.Models.Domain
{
    public class ExecutedTasksModel
    {
        [JsonPropertyName("Id")]
        public Guid RobotId { get; set; }

        [JsonPropertyName("TaskId")]
        public Guid Id { get; set; }


        //Probably outdated, to be decided if there is a usecase for this model. 08.02.2023 Radu
    }
}