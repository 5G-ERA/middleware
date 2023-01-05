using Middleware.Common.Enums;
using Middleware.Common.Models;
using Middleware.DataAccess.Repositories.Abstract;

namespace Middleware.DataAccess.Repositories
{
    internal class RedisRepository<T> : IBaseRepository<T> where T : class
    {
        public Task<T> AddAsync(T model)
        {
            throw new NotImplementedException();
        }

        public Task<T> AddAsync(T model, Func<Guid> guidProvider)
        {
            throw new NotImplementedException();
        }

        public Task<bool> AddGraphAsync(GraphEntityModel model)
        {
            throw new NotImplementedException();
        }

        public Task<bool> AddRelationAsync(RelationModel relation)
        {
            throw new NotImplementedException();
        }

        public Task<bool> DeleteByIdAsync(Guid id)
        {
            throw new NotImplementedException();
        }

        public Task<bool> DeleteGraphModelAsync(GraphEntityModel model)
        {
            throw new NotImplementedException();
        }

        public Task<bool> DeleteRelationAsync(RelationModel relation)
        {
            throw new NotImplementedException();
        }

        public Task<List<T>> GetAllAsync()
        {
            throw new NotImplementedException();
        }

        public Task<T> GetByIdAsync(Guid id)
        {
            throw new NotImplementedException();
        }

        public Task<List<RelationModel>> GetRelation(Guid id, string relationName, RelationDirection direction = RelationDirection.Outgoing)
        {
            throw new NotImplementedException();
        }

        public Task<List<RelationModel>> GetRelations(Guid id, List<string> relationNames)
        {
            throw new NotImplementedException();
        }
    }
}
