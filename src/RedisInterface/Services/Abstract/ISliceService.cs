using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Services.Abstract;

internal interface ISliceService
{
    Task ReRegisterSlicesAsync(IReadOnlyList<SliceModel> slices, Location location = null);

    Task<List<SliceModel>> GetAllSlicesAsync();

    Task<List<RelationModel>> GetRelationAsync(Guid id, string name,
        RelationDirection direction = RelationDirection.Outgoing);

    Task<SliceModel> GetByIdAsync(Guid id);

    Task<SliceModel> GetBySliceIdAsync(string id);

    Task SliceAddEmbb(SliceModel embbSlice);

    Task SliceAddUrllc(SliceModel urllcSlice);

    Task SliceUpdateEmbb(SliceModel embbSlice);

    Task SliceUpdateUrllc(SliceModel urllcSlice);

    Task<bool> SliceDeleteEmbb(Guid id);

    Task<bool> SliceDeleteUrllc(Guid id);
}