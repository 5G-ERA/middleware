namespace Middleware.Common.Models
{
    public class RelationModel
    {
        public GraphEntityModel InitiatesFrom { get; set; } = new GraphEntityModel();
        public string RelationName { get; set; }
        public List <KeyValuePair> RelationAttributes { get; set; } = new List<KeyValuePair>();
        public GraphEntityModel PointsTo { get; set; } = new GraphEntityModel();

        public RelationModel() { }

        public RelationModel(GraphEntityModel initiatesFrom, GraphEntityModel pointsTo, string relationName, List<KeyValuePair> relationAttributes = null)
        {
            InitiatesFrom = initiatesFrom;
            PointsTo = pointsTo;
            RelationName = relationName.ToUpper();
            if (relationAttributes is not null)
            {
                RelationAttributes = relationAttributes;
            }
        }
    }
}
