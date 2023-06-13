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

    Task SliceAddEmbb(SliceModel embbSlice);

    Task SliceAddUrllc(SliceModel urllcSlice);
}