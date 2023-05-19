using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;

namespace Middleware.RedisInterface.Services.Abstract;

public interface ISliceService
{
    Task ReRegisterSlices(IReadOnlyList<SliceModel> slices, Location location = null);
}