using Middleware.DataAccess.Repositories.Abstract;
using Redis.OM;
using Redis.OM.Searching;
using System.Linq.Expressions;
using Middleware.Common.Models;

namespace Middleware.DataAccess.Repositories
{
    public class RedisRepository<T, TDto> : IRedisRepository<T> where T : BaseModel where TDto : Dto.Dto
    {
        protected readonly IRedisCollection<T> Collection;

        public RedisRepository(RedisConnectionProvider provider)
        {
            Collection = provider.RedisCollection<T>();
        }
        public async Task<T> AddAsync(T model)
        {
            var dto = (TDto) model.T
            var id = await Collection.InsertAsync(model);
            
            return model;
        }

        public async Task DeleteAsync(T model)
        {
            await Collection.DeleteAsync(model);
        }

        public async Task<List<T>> GetAllAsync()
        {
            IList<T> listAsync = await Collection.ToListAsync();
            return listAsync.ToList();
        }

        public async Task<T?> GetByIdAsync(Guid id)
        {
           return await Collection.FindByIdAsync(id.ToString());
        }

        public async Task<T?> FindSingleAsync(Expression<Func<T, bool>> predicate)
        {
            return await Collection.SingleOrDefaultAsync(predicate);
        }

        public async Task<List<T>> FindAsync(Expression<Func<T, bool>> predicate)
        {
            IList<T> list = await Collection.Where(predicate).ToListAsync();
            return list.ToList();
        }

        public IRedisCollection<T> FindQuery(Expression<Func<T, bool>> predicate)
        {
            return Collection.Where(predicate);
        }
    }
}
