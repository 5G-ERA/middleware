using Microsoft.Extensions.Hosting;
using Middleware.Models.Dto;
using Redis.OM;

namespace Middleware.DataAccess.HostedServices;

public class IndexCreationService : IHostedService
{
    private readonly RedisConnectionProvider _provider;

    public IndexCreationService(RedisConnectionProvider provider)
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
    }

    public Task StopAsync(CancellationToken cancellationToken)
    {
        return Task.CompletedTask;
    }
}
