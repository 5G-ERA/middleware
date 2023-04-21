using Middleware.Common.Enums;
using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IRelationRepository
    {
        /// <summary>
        /// Get relation for the specified object by the name of the relation
        /// </summary>
        /// <param name="id">Identifier of the object</param>
        /// <param name="relationName">Name of the relation</param>
        /// <param name="direction">Direction of the relation, outgoing from the object or incoming to the object</param>
        /// <returns></returns>
        Task<List<RelationModel>> GetRelation(Guid id, string relationName, RelationDirection direction = RelationDirection.Outgoing);
        /// <summary>
        /// Get relations for the specified object by the names of the relations
        /// </summary>
        /// <param name="id">Identifier of the object</param>
        /// <param name="relationNames">Names of the relations</param>
        /// <returns></returns>
        Task<List<RelationModel>> GetRelations(Guid id, List<string> relationNames);

        Task<bool> AddGraphAsync(GraphEntityModel model);
        Task<bool> AddRelationAsync(RelationModel relation);

        Task<bool> DeleteGraphModelAsync(GraphEntityModel model);

        Task<bool> DeleteRelationAsync(RelationModel relation);

        Task<Dictionary<string, List<RedisGraphResult>>> GetAllRelations();
    }
}
