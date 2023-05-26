using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;

namespace Middleware.RedisInterface.Services.Abstract;

internal interface ISliceService
{
    Task ReRegisterSlices(IReadOnlyList<SliceModel> slices, Location location = null);

    Task<List<SliceModel>> GetAllSlices();

    Task<List<RelationModel>> GetRelationAsync(Guid id, string name,
        RelationDirection direction = RelationDirection.Outgoing);
}