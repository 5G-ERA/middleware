﻿using Redis.OM.Modeling;

namespace Middleware.Models.Dto
{
    [Document(StorageType = StorageType.Json, IndexName = "dialogue-idx", Prefixes = new[] { "Dialogue" })]
    public class DialogueDto : Dto
    {
        [Indexed]
        [RedisIdField]
        public override string Id { get; init; } = default!;
        /// <summary>
        /// Question / Name
        /// </summary>
        [Searchable]
        public string? Question { get; set; }
        [Indexed]
        public bool IsSingleAnswer { get; set; }
    }
}