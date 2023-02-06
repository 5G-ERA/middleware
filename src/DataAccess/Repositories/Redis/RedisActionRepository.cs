using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM;
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

        if (!string.IsNullOrEmpty(patch.Order.ToString()))
        {
            currentModel.Order = patch.Order;
        }
        if (!string.IsNullOrEmpty(patch.Placement))
        {
            currentModel.Placement = patch.Placement;
        }
        if (!string.IsNullOrEmpty(patch.ActionPriority))
        {
            currentModel.ActionPriority = patch.ActionPriority;
        }
        await UpdateAsync(currentModel);
        return currentModel;
    }
}
