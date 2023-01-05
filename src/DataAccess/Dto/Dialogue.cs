using Redis.OM.Modeling;

namespace Middleware.DataAccess.Dto
{
    [Document(StorageType = StorageType.Json, IndexName = "dialogue-idx", Prefixes = new[] { "Dialogue" })]
    internal class DialogueDto
    {
        [Indexed]
        public string? Id { get; set; }
        /// <summary>
        /// Question / Name
        /// </summary>
        [Indexed]
        [Searchable]
        public string? Question { get; set; }
        [Indexed]
        public bool IsSingleAnswer { get; set; }
    }
}
