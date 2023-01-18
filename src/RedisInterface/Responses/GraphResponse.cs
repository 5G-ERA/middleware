using Middleware.Common.Models;

namespace Middleware.RedisInterface.Responses;

public class GraphResponse
{
    public List<GraphEntityModel> Entities { get; set; }

    public List<SimpleRelation> Relations { get; set; }
}