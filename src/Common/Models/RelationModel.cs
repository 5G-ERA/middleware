using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    public class RelationModel
    {
        public GraphEntityModel InitiatesFrom { get; set; }
        public string RelationName { get; set; }
        public List <KeyValuePair> RelationAttributes { get; set; } = new List<KeyValuePair>();
        public GraphEntityModel PointsTo { get; set; }


    }
}
