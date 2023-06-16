using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Services;

public class SliceService : ISliceService
{
    private readonly ICloudRepository _cloudRepository;
    private readonly IEdgeRepository _edgeRepository;
    private readonly ILogger _logger;
    private readonly IOptions<MiddlewareConfig> _middlewareConfig;
    private readonly ISliceRepository _sliceRepository;

    public SliceService(IEdgeRepository edgeRepository, ICloudRepository cloudRepository,
        ISliceRepository sliceRepository, IOptions<MiddlewareConfig> middlewareConfig, ILogger<SliceService> logger)
    {
        _edgeRepository = edgeRepository;
        _cloudRepository = cloudRepository;
        _sliceRepository = sliceRepository;
        _middlewareConfig = middlewareConfig;
        _logger = logger;
    }

    /// <summary>
    ///     Re Registers (deletes and adds new) Slices associated with the specified Location. When Location is not specified,
    ///     the current location is used.
    /// </summary>
    /// <param name="slices"></param>
    /// <param name="location"></param>
    /// <returns></returns>
    public async Task ReRegisterSlicesAsync(IReadOnlyList<SliceModel> slices, Location location = null)
    {
        var locationData = location is null
            ? await GetCurrentLocation()
            : await GetLocationData(location);

        await DeleteExistingSlices(locationData);
        await AddNewSliceDefinitions(slices, locationData);
    }

    /// <inheritdoc />
    public Task<List<SliceModel>> GetAllSlicesAsync()
    {
        return _sliceRepository.GetAllAsync();
    }

    /// <inheritdoc />
    public Task<List<RelationModel>> GetRelationAsync(Guid id, string name,
        RelationDirection direction = RelationDirection.Outgoing)
    {
        return _sliceRepository.GetRelation(id, name, direction);
    }

    /// <inheritdoc />
    public Task<SliceModel> GetByIdAsync(Guid id)
    {
        return _sliceRepository.GetByIdAsync(id);
    }

    /// <summary>
    ///     Add new slice and connect it to the current Middleware location
    /// </summary>
    /// <param name="slice"></param>
    /// <returns></returns>
    public async Task SliceAddAsync(SliceModel slice)
    {
        await _sliceRepository.AddAsync(slice);

        var location = await GetCurrentLocation();
        var relation = new RelationModel
        {
            InitiatesFrom = new(location.Id, location.Name, location.GetType()),
            RelationName = "OFFERS"
        };
        var pointsTo = new GraphEntityModel(slice.Id, slice.Name, slice.GetType());
        relation.PointsTo = pointsTo;

        await _sliceRepository.AddAsync(slice);
        if (location.GetType() == typeof(CloudModel))
            await _cloudRepository.AddRelationAsync(relation);
        else if (location.GetType() == typeof(EdgeModel))
            await _edgeRepository.AddRelationAsync(relation);
    }

    /// <summary>
    ///     Update slice
    /// </summary>
    /// <param name="embbSlice"></param>
    /// <returns></returns>
    /// <exception cref="NotImplementedException"></exception>
    public async Task SliceUpdateAsync(SliceModel embbSlice)
    {
        await _sliceRepository.UpdateAsync(embbSlice);
    }

    /// <summary>
    ///     Delete slice by Id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    /// <exception cref="NotImplementedException"></exception>
    public async Task<bool> SliceDeleteAsync(Guid id)
    {
        try
        {
            var location = await GetCurrentLocation();
            await DeleteExistingSlice(location, id);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "there was an error during the deletion of the Slice with id: {0}", id);
        }

        return true;
    }

    /// <summary>
    ///     Get Slice by its internal SliceId
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    public async Task<SliceModel> GetBySliceIdAsync(string id)
    {
        return await _sliceRepository.FindSingleAsync(slice => slice.Name == id);
    }

    public async Task<bool> DeleteBySliceId(string sliceId)
    {
        var location = await GetCurrentLocation();
        var existingSlice = await GetBySliceIdAsync(sliceId);

        if (existingSlice is null) return false;

        await DeleteExistingSlice(location, existingSlice.Id);
        return true;
    }

    /// <summary>
    ///     Adds a new Slice definitions and links them with the specified location.
    /// </summary>
    /// <param name="slices"></param>
    /// <param name="location"></param>
    /// <returns></returns>
    private async Task AddNewSliceDefinitions(IReadOnlyList<SliceModel> slices, BaseModel location)
    {
        var relation = new RelationModel
        {
            InitiatesFrom = new(location.Id, location.Name, location.GetType()),
            RelationName = "OFFERS"
        };
        foreach (var slice in slices)
        {
            var pointsTo = new GraphEntityModel(slice.Id, slice.Name, slice.GetType());
            relation.PointsTo = pointsTo;

            await _sliceRepository.AddAsync(slice);
            if (location.GetType() == typeof(CloudModel))
                await _cloudRepository.AddRelationAsync(relation);
            else if (location.GetType() == typeof(EdgeModel))
                await _edgeRepository.AddRelationAsync(relation);

            relation.PointsTo = null!;
        }
    }

    private async Task DeleteExistingSlice(BaseModel location, Guid sliceId)
    {
        if (location.GetType() == typeof(CloudModel))
        {
            var relations = await _cloudRepository.GetRelation(location.Id, "OFFERS");
            // should be only one
            foreach (var relation in relations.Where(r => r.PointsTo.Id == sliceId))
            {
                await _cloudRepository.DeleteRelationAsync(relation);
                await _sliceRepository.DeleteByIdAsync(relation.PointsTo.Id);
            }
        }
        else if (location.GetType() == typeof(EdgeModel))
        {
            var relations = await _edgeRepository.GetRelation(location.Id, "OFFERS");
            // should be only one
            foreach (var relation in relations.Where(r => r.PointsTo.Id == sliceId))
            {
                await _edgeRepository.DeleteRelationAsync(relation);
                await _sliceRepository.DeleteByIdAsync(relation.PointsTo.Id);
            }
        }
    }

    /// <summary>
    ///     Deletes all the slices associated with the specified location.
    /// </summary>
    /// <param name="location"></param>
    /// <returns></returns>
    private async Task DeleteExistingSlices(BaseModel location)
    {
        if (location.GetType() == typeof(CloudModel))
        {
            var relations = await _cloudRepository.GetRelation(location.Id, "OFFERS");

            foreach (var relation in relations)
            {
                await _cloudRepository.DeleteRelationAsync(relation);
                await _sliceRepository.DeleteByIdAsync(relation.PointsTo.Id);
            }
        }
        else if (location.GetType() == typeof(EdgeModel))
        {
            var relations = await _edgeRepository.GetRelation(location.Id, "OFFERS");

            foreach (var relation in relations)
            {
                await _edgeRepository.DeleteRelationAsync(relation);
                await _sliceRepository.DeleteByIdAsync(relation.PointsTo.Id);
            }
        }
    }

    /// <summary>
    ///     Get current location data from configuration
    /// </summary>
    /// <returns></returns>
    private async Task<BaseModel> GetCurrentLocation()
    {
        var config = _middlewareConfig.Value;
        BaseModel location;
        if (config.InstanceType == LocationType.Cloud.ToString())
            location = await _cloudRepository.GetCloudResourceDetailsByNameAsync(config.InstanceName);
        else
            location = await _edgeRepository.GetEdgeResourceDetailsByNameAsync(config.InstanceName);

        return location;
    }

    /// <summary>
    ///     Get specified location data
    /// </summary>
    /// <param name="location"></param>
    /// <returns></returns>
    private async Task<BaseModel> GetLocationData(Location location)
    {
        BaseModel locationData;
        if (location.Type == LocationType.Cloud)
            locationData = await _cloudRepository.GetCloudResourceDetailsByNameAsync(location.Name);
        else
            locationData = await _edgeRepository.GetEdgeResourceDetailsByNameAsync(location.Name);

        return locationData;
    }
}