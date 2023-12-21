using Middleware.CentralApi.Contracts.Responses;
using Middleware.Models.Domain;

namespace Middleware.CentralApi.Services.Abstract;

public interface IRobotService
{
    Task<List<string>> CreateRelation(RelationToLocationRequest data);
    Task<List<string>> DeleteRelation(RelationToLocationRequest data);
}
