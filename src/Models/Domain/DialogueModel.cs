using System.Text.Json.Serialization;
using Middleware.Models.Dto;

namespace Middleware.Models.Domain;

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
    
    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new DialogueDto()
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            IsSingleAnswer = domain.IsSingleAnswer,
            Answer = domain.Answer.ToList(),

        };
    }
}