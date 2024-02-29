using Middleware.CentralApi.Contracts.Responses;

namespace Middleware.CentralApi.Services;

public interface IRobotService
{
    Task<List<string>> CreateRelation(RelationToLocationRequest data);
    Task<List<string>> DeleteRelation(RelationToLocationRequest data);
}
