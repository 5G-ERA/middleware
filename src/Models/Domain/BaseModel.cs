namespace Middleware.Models.Domain
{
    public abstract class BaseModel
    {
        public abstract Guid Id { get; set; }

        public abstract string Name { get; set; }

        public List<RelationModel>? Relations { get; set; } //= new List<RelationModel>();

        public abstract Dto.Dto ToDto();
    }
}