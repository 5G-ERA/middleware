using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;

namespace Middleware.RedisInterface.Services.Abstract;

public interface ISliceService
{
    /// <summary>
    ///     Removes existing Slice association with the location and registers new slices
    /// </summary>
    /// <param name="slices">List of slices to be associated with the location</param>
    /// <param name="location">Location for the slices to be associated. Default: current location</param>
    /// <returns></returns>
    Task ReRegisterSlicesAsync(IReadOnlyList<SliceModel> slices, Location location = null);

    /// <summary>
    ///     Get All Slice definitions
    /// </summary>
    /// <returns></returns>
    Task<List<SliceModel>> GetAllSlicesAsync();

    /// <summary>
    ///     Get the relation between Slice and other objects
    /// </summary>
    /// <param name="id"></param>
    /// <param name="name"></param>
    /// <param name="direction"></param>
    /// <returns></returns>
    Task<List<RelationModel>> GetRelationAsync(Guid id, string name,
        RelationDirection direction = RelationDirection.Outgoing);

    /// <summary>
    ///     Get Slice by Id
    /// </summary>
    /// <param name="id">Middleware slice Id</param>
    /// <returns></returns>
    Task<SliceModel> GetByIdAsync(Guid id);

    /// <summary>
    ///     Get Slice by testbed Slice Id (slice name in Middleware)
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    Task<SliceModel> GetBySliceIdAsync(string id);

    /// <summary>
    ///     Add new slice in current location
    /// </summary>
    /// <param name="slice"></param>
    /// <returns></returns>
    Task SliceAddAsync(SliceModel slice);

    /// <summary>
    ///     Update existing slice definition
    /// </summary>
    /// <param name="embbSlice"></param>
    /// <returns></returns>
    Task SliceUpdateAsync(SliceModel embbSlice);

    /// <summary>
    ///     Delete existing slice by Middleware Id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    Task<bool> SliceDeleteAsync(Guid id);

    /// <summary>
    ///     Delete Slice by testbed SliceId (name in Middleware)
    /// </summary>
    /// <param name="sliceId"></param>
    /// <returns></returns>
    Task<bool> DeleteBySliceId(string sliceId);
}