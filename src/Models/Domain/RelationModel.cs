namespace Middleware.Models.Domain;

public class RelationModel
{
    public GraphEntityModel InitiatesFrom { get; set; } = new();
    public string RelationName { get; set; } = default!;
    public List<KeyValuePair>? RelationAttributes { get; set; } = new();
    public GraphEntityModel PointsTo { get; set; } = new();

    public RelationModel()
    {
    }

    public RelationModel(GraphEntityModel initiatesFrom, GraphEntityModel pointsTo, string relationName,
        List<KeyValuePair>? relationAttributes = null)
    {
        InitiatesFrom = initiatesFrom;
        PointsTo = pointsTo;
        RelationName = relationName.ToUpper();
        if (relationAttributes is not null) RelationAttributes = relationAttributes;
    }
}