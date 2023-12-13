using Microsoft.Extensions.Hosting;
using Middleware.Models.Dto;
using Middleware.Models.Dto.Slice;
using Redis.OM;
using Redis.OM.Contracts;

namespace Middleware.DataAccess.HostedServices;

public class IndexCreationService : BackgroundService
{
    private readonly IRedisConnectionProvider _provider;

    public IndexCreationService(IRedisConnectionProvider provider)
    {
        _provider = provider;
    }

    /// <inheritdoc />
    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        await RecreateIndexes();
    }

    /// <summary>
    ///     Checks redis to see if the index already exists, if it doesn't create a new index
    /// </summary>
    /// <param name="cancellationToken"></param>
    public async Task StartAsync(CancellationToken cancellationToken)
    {
    }

    public Task StopAsync(CancellationToken cancellationToken)
    {
        return Task.CompletedTask;
    }

    private async Task RecreateIndexes()
    {
        var info = (await _provider.Connection.ExecuteAsync("FT._LIST")).ToArray().Select(x => x.ToString()).ToList();
        if (info.All(x => x != "action-idx"))
            await _provider.Connection.CreateIndexAsync(typeof(ActionDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(ActionDto));
            await _provider.Connection.CreateIndexAsync(typeof(ActionDto));
        }

        if (info.Any(x => x == "robot-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(RobotDto));
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }

        if (info.Any(x => x == "actionPlan-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(ActionPlanDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(ActionPlanDto));
            await _provider.Connection.CreateIndexAsync(typeof(ActionPlanDto));
        }

        if (info.Any(x => x == "cloud-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(CloudDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(CloudDto));
            await _provider.Connection.CreateIndexAsync(typeof(CloudDto));
        }

        if (info.Any(x => x == "containerImage-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(ContainerImageDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(ContainerImageDto));
            await _provider.Connection.CreateIndexAsync(typeof(ContainerImageDto));
        }

        if (info.Any(x => x == "dialogue-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(DialogueDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(DialogueDto));
            await _provider.Connection.CreateIndexAsync(typeof(DialogueDto));
        }

        if (info.Any(x => x == "edge-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(EdgeDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(EdgeDto));
            await _provider.Connection.CreateIndexAsync(typeof(EdgeDto));
        }

        if (info.Any(x => x == "instance-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(InstanceDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(InstanceDto));
            await _provider.Connection.CreateIndexAsync(typeof(InstanceDto));
        }

        if (info.Any(x => x == "policy-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(PolicyDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(PolicyDto));
            await _provider.Connection.CreateIndexAsync(typeof(PolicyDto));
        }

        if (info.Any(x => x == "task-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(TaskDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(TaskDto));
            await _provider.Connection.CreateIndexAsync(typeof(TaskDto));
        }

        if (info.Any(x => x == "user-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(UserDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(UserDto));
            await _provider.Connection.CreateIndexAsync(typeof(UserDto));
        }

        if (info.Any(x => x == "robotStatus-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(RobotStatusDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(RobotStatusDto));
            await _provider.Connection.CreateIndexAsync(typeof(RobotStatusDto));
        }

        if (info.Any(x => x == "netAppStatus-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(NetAppStatusDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(NetAppStatusDto));
            await _provider.Connection.CreateIndexAsync(typeof(NetAppStatusDto));
        }

        if (info.Any(x => x == "slice-idx") == false)
            await _provider.Connection.CreateIndexAsync(typeof(SliceDto));
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(SliceDto));
            await _provider.Connection.CreateIndexAsync(typeof(SliceDto));
        }
        if (info.All(x => x != "location-idx"))
        {
            await _provider.Connection.CreateIndexAsync(typeof(LocationDto));
        }
        else
        {
            await _provider.Connection.DropIndexAsync(typeof(LocationDto));
            await _provider.Connection.CreateIndexAsync(typeof(LocationDto));
        }
    }
}