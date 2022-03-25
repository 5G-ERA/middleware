using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class DialogueModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Question")]
    public string Question { get; set; }

    [JsonPropertyName("IsSingleAnswer")]
    public bool IsSingleAnswer { get; set; }

    [JsonPropertyName("Answer")]
    public List<KeyValuePair> Answer { get; set; }

    public DateTime TimeStamp { get; set; }
}