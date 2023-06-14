using Middleware.Models.Domain.Slice;
using Middleware.Models.Dto.Slice;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface ISliceRepository : IRedisRepository<SliceModel, SliceDto>, IRelationRepository
{
}