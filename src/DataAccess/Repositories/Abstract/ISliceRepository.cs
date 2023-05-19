using Middleware.Models.Domain.Slice;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface ISliceRepository : IBaseRepository<SliceModel>, IRelationRepository
{
}