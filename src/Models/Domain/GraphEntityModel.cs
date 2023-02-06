using Middleware.Models.Enums;
using Middleware.Models.ExtensionMethods;

namespace Middleware.Models.Domain
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
            Type = dbIndex.ToString().ToUpper();
        }

        public GraphEntityModel(Guid id, RedisDbIndexEnum dbIndex)
        {
            Id = id;
            Type = dbIndex.ToString().ToUpper();
        }
        public GraphEntityModel(Guid id, string name, string type)
        {
            Id = id;
            Name = name;
            Type = type;
        }

        public GraphEntityModel(Guid id, string name, Type type)
        {
            Id = id;
            Name = name;
            Type = type.GetModelName();
        }
        public GraphEntityModel(Guid id, string name, string entityName)
        {
            Id = id;
            Name = name;
            Type = entityName;
        }
    }
}