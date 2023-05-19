using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Services;

public class SliceService : ISliceService
{
    private readonly ICloudRepository _cloudRepository;
    private readonly IEdgeRepository _edgeRepository;
    private readonly ISliceRepository _sliceRepository;

    public SliceService(IEdgeRepository edgeRepository, ICloudRepository cloudRepository,
        ISliceRepository sliceRepository)
    {
        _edgeRepository = edgeRepository;
        _cloudRepository = cloudRepository;
        _sliceRepository = sliceRepository;
    }

    public Task<bool> ReRegisterSlices(IReadOnlyList<SliceModel> slices, Location location = null)
    {
        throw new NotImplementedException();
    }
}