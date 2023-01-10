using Middleware.DataAccess.Repositories.Abstract;
using Redis.OM;
using Redis.OM.Searching;
using System.Linq.Expressions;

namespace Middleware.DataAccess.Repositories
{
    public class RedisRepository<T> : IRedisRepository<T> where T : Dto.Dto
    {
        public IRedisCollection<T> _collection;

        public RedisRepository(RedisConnectionProvider provider)
        {
            _collection = provider.RedisCollection<T>();
        }
        public async Task<T> AddAsync(T model)
        {
            var id = await _collection.InsertAsync(model);
            
            return model;
        }

        public async Task DeleteAsync(T model)
        {
            await _collection.DeleteAsync(model);
        }

        public async Task<List<T>> GetAllAsync()
        {
            IList<T> listAsync = await _collection.ToListAsync();
            return listAsync.ToList();
        }

        public async Task<T?> GetByIdAsync(Guid id)
        {
           return await _collection.FindByIdAsync(id.ToString());
        }

        public async Task<T?> FindSingleAsync(Expression<Func<T, bool>> predicate)
        {
            return await _collection.SingleOrDefaultAsync(predicate);
        }

        public async Task<List<T>> FindAsync(Expression<Func<T, bool>> predicate)
        {
            IList<T> list = await _collection.Where(predicate).ToListAsync();
            return list.ToList();
        }

        public IRedisCollection<T> FindQuery(Expression<Func<T, bool>> predicate)
        {
            return _collection.Where(predicate);
        }
    }
}
