using Middleware.Models.Domain;
using Redis.OM.Modeling;
using KeyValuePair = Middleware.Models.Domain.KeyValuePair;

namespace Middleware.Models.Dto
{
    [Document(StorageType = StorageType.Json, IndexName = "dialogue-idx", Prefixes = new[] { "Dialogue" })]
    public class DialogueDto : Dto
    {
        [Indexed]
        [RedisIdField]
        public override string Id { get; set; } = default!;
        /// <summary>
        /// Question / Name
        /// </summary>
        [Searchable]
        public string? Name { get; set; }
        [Indexed]
        public bool IsSingleAnswer { get; set; }

        [Indexed]
        public List<KeyValuePair> Answer { get; set; }
        
        public override BaseModel ToModel()
        {
            var dto = this;
            return new DialogueModel()
            {
                Id = Guid.Parse(dto.Id!),
                Name = dto.Name,
                IsSingleAnswer = dto.IsSingleAnswer,
                Answer = dto.Answer
            };
        }
    }
}