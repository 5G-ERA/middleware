﻿using Middleware.Common.Enums;

namespace Middleware.Common.Models
{
    public class GraphEntityModel
    {
        public Guid Id { get; set; }
        public string Type { get; set; }

        public string Name { get; set; }

        public GraphEntityModel()
        {
            
        }

        public GraphEntityModel(Guid id, string name, RedisDbIndexEnum dbIndex)
        {
            Id = id;
            Name = name;

        }
    }
}
