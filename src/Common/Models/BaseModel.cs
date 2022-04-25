﻿namespace Middleware.Common.Models
{
    public abstract class BaseModel
    {
        public abstract Guid Id { get; set; }

        public abstract string Name { get; set; }
        public List<RelationModel> Relations { get; set; } = new List<RelationModel>();

    }
}
