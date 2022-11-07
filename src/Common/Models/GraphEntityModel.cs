using Middleware.Common.Enums;

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
            Type = dbIndex.ToString().ToUpper();
        }

        public GraphEntityModel(Guid id, RedisDbIndexEnum dbIndex)
        {
            Id = id;
            Type = dbIndex.ToString().ToUpper();
        }

        public GraphEntityModel(Guid id, string name, Type type)
        {
            Id = id;
            Name = name;
            Type = type.Name.EndsWith("Model")
                ? Type = type.Name.Remove(type.Name.LastIndexOf("M", StringComparison.Ordinal))
                : type.Name;
        }
    }
}
