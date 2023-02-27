namespace Middleware.RedisInterface.Contracts.Responses;

public class SimpleRelation
{
    public Guid OriginatingId { get; set; }
    public string RelationName { get; set; }
    public Guid PointsToId { get; set; }
}
public class SimpleRelationComparer : IEqualityComparer<SimpleRelation>
{
    public bool Equals(SimpleRelation x, SimpleRelation y)
    {
        if (ReferenceEquals(x, y)) return true;
        if (ReferenceEquals(x, null)) return false;
        if (ReferenceEquals(y, null)) return false;
        if (x.GetType() != y.GetType()) return false;
        return  x.RelationName == y.RelationName  
             && (x.OriginatingId.Equals(y.PointsToId) && x.PointsToId.Equals(y.PointsToId)
                 || x.OriginatingId.Equals(y.PointsToId) && y.PointsToId.Equals(x.OriginatingId));
    }

    public int GetHashCode(SimpleRelation obj)
    {
        var elems = new[] { obj.OriginatingId.ToString(), obj.PointsToId.ToString(), obj.RelationName };
        return string.Join("", elems.OrderBy(x => x).ToArray()).GetHashCode();
    }
}
