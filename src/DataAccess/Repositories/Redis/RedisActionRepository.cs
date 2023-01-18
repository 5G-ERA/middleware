using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Common.Models;
using Middleware.DataAccess.Dto;
using Middleware.DataAccess.Repositories.Abstract;
using Redis.OM;
using RedisGraphDotNet.Client;

namespace Middleware.DataAccess.Repositories.Redis;

public class RedisActionRepository : RedisRepository<ActionModel, ActionDto>
{
    public RedisActionRepository(RedisConnectionProvider provider, IRedisGraphClient redisGraph, string entityName, bool isWritableToGraph) : base(provider, redisGraph, "Action", true)
    {

    }

    public async Task<ActionModel> PatchActionAsync(Guid id, ActionModel patch)
    {
        ActionDto? model = await GetByIdAsync(id);
        if(model is null)
            return null;
        ActionModel currentModel = model.ToActionModel();
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
        await UpdateAsync(currentModel.ToActionDto());        
        return currentModel;
    }
}
