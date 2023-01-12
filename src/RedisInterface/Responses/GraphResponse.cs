using Middleware.Common.Models;

namespace Middleware.RedisInterface.Responses;

public class GraphResponse
{
    public List<GraphEntityModel> Entities { get; set; }

    public List<SimpleRelation> Relations { get; set; }
}
public class SimpleRelation
{
    public Guid OriginatingId { get; set; }
    public string RelationName { get; set; }
    public Guid PointsToId { get; set; }
}
