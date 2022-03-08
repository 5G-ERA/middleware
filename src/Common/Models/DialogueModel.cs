using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class DialogueModel
{
    [JsonPropertyName("Question_Id")]
    public Guid QuestionId { get; set; }

    [JsonPropertyName("Question")]
    public string Question { get; set; }

    [JsonPropertyName("Answer")]
    public string Answer { get; set; }
}