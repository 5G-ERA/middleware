namespace Middleware.Common.Models
{
    public class RelationModel
    {
        public GraphEntityModel InitiatesFrom { get; set; } = new GraphEntityModel();
        public string RelationName { get; set; }
        public List <KeyValuePair> RelationAttributes { get; set; } = new List<KeyValuePair>();
        public GraphEntityModel PointsTo { get; set; } = new GraphEntityModel();

        
    }
}
