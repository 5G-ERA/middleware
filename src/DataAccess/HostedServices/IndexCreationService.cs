using Microsoft.Extensions.Hosting;
using Middleware.Models.Dto;
using Redis.OM;
using Redis.OM.Contracts;

namespace Middleware.DataAccess.HostedServices;

public class IndexCreationService : IHostedService
{
    private readonly IRedisConnectionProvider _provider;

    public IndexCreationService(IRedisConnectionProvider provider)
    {
        _provider = provider;
    }

    /// <summary>
    /// Checks redis to see if the index already exists, if it doesn't create a new index
    /// </summary>
    /// <param name="cancellationToken"></param>
    public async Task StartAsync(CancellationToken cancellationToken)
    {
        var info = (await _provider.Connection.ExecuteAsync("FT._LIST")).ToArray().Select(x => x.ToString()).ToList();
        if (info.All(x => x != "action-idx"))
        {
            await _provider.Connection.CreateIndexAsync(typeof(ActionDto));
        }
        if (info.Any(x => x == "robot-idx") == false)
        {
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }
        if (info.Any(x => x == "actionPlan-idx") == false)
        {
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }
        if (info.Any(x => x == "cloud-idx") == false)
        {
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }
        if (info.Any(x => x == "containerImage-idx") == false)
        {
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }
        if (info.Any(x => x == "dialogue-idx") == false)
        {
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }
        if (info.Any(x => x == "edge-idx") == false)
        {
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }
        if (info.Any(x => x == "instance-idx") == false)
        {
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }
        if (info.Any(x => x == "policy-idx") == false)
        {
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }
        if (info.Any(x => x == "task-idx") == false)
        {
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }
        if (info.Any(x => x == "user-idx") == false)
        {
            await _provider.Connection.CreateIndexAsync(typeof(RobotDto));
        }
    }

    public Task StopAsync(CancellationToken cancellationToken)
    {
        return Task.CompletedTask;
    }
}
