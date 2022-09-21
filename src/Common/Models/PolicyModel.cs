using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class PolicyModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; }

    [JsonPropertyName("Timestamp")]
    public DateTime Timestamp { get; set; }

    [JsonPropertyName("IsActive")]
    public bool? IsActive { get; set; }

    [JsonPropertyName("Description")]
    public string Description { get; set; }

    [JsonPropertyName("CannotCoexistFamily")]
    public int CannotCoexistFamily { get; set; }
}