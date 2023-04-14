using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories;

public class RedisActionRepository : RedisRepository<ActionModel, ActionDto>, IActionRepository
{
    public RedisActionRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(provider, redisGraph, true, logger)
    {

    }

    public async Task<ActionModel> PatchActionAsync(Guid id, ActionModel patch)
    {
        ActionModel? currentModel = await GetByIdAsync(id);
        if (currentModel == null)
        {
            return null;
        }
        if (!string.IsNullOrEmpty(patch.Name))
        {
            currentModel.Name = patch.Name;
        }
        if (patch.Tags != null)
        {
            currentModel.Tags = patch.Tags;
        }
        if (!string.IsNullOrEmpty(patch.Order.ToString()))
        {
            currentModel.Order = patch.Order;
        }
        if (!string.IsNullOrEmpty(patch.Placement))
        {
            currentModel.Placement = patch.Placement;
        }
        if (patch.PlacementType is not null)
        {
            currentModel.PlacementType = patch.PlacementType;
        }
        if (!string.IsNullOrEmpty(patch.ActionPriority))
        {
            currentModel.ActionPriority = patch.ActionPriority;
        }
        if (!string.IsNullOrEmpty(patch.ActionStatus))
        {
            currentModel.ActionStatus = patch.ActionStatus;
        }
        if (patch.Services != null)
        {
            currentModel.Services = patch.Services;
        }
        if (!string.IsNullOrEmpty(patch.MinimumRam.ToString()))
        {
            currentModel.MinimumRam = patch.MinimumRam;
        }
        if (!string.IsNullOrEmpty(patch.MinimumNumCores.ToString()))
        {
            currentModel.MinimumNumCores = patch.MinimumNumCores;
        }
        await UpdateAsync(currentModel);
        return currentModel;
    }
}
