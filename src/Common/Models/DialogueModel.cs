using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class DialogueModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }
    /// <summary>
    /// Question / Name
    /// </summary>
    [JsonPropertyName("Question")]
    public override string Name { get; set; }

    [JsonPropertyName("IsSingleAnswer")]
    public bool IsSingleAnswer { get; set; }

    [JsonPropertyName("Answer")]
    public List<KeyValuePair> Answer { get; set; }

}
