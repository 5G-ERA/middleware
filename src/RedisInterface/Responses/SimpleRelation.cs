namespace Middleware.RedisInterface.Responses;

public class SimpleRelation
{
    public Guid OriginatingId { get; set; }
    public string RelationName { get; set; }
    public Guid PointsToId { get; set; }
}
