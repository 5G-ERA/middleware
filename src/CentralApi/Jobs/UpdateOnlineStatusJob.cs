using Middleware.Common.Job;
using Middleware.DataAccess.Repositories.Abstract;
using Quartz;

namespace Middleware.CentralApi.Jobs;

[DisallowConcurrentExecution]
public class UpdateOnlineStatusJob : BaseJob<UpdateOnlineStatusJob>
{
    private readonly ILocationRepository _locationRepository;
    private readonly ISystemConfigRepository _systemConfig;

    public UpdateOnlineStatusJob(ILogger<UpdateOnlineStatusJob> logger, ILocationRepository locationRepository,
        ISystemConfigRepository systemConfig) : base(logger)
    {
        _locationRepository = locationRepository;
        _systemConfig = systemConfig;
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        var cfg = await _systemConfig.GetConfigAsync();
        if (cfg is null) throw new ArgumentException("No system config was found");

        var heartbeatExpiration = cfg.HeartbeatExpirationInMinutes;
        try
        {
            var locations = await _locationRepository.GetAllAsync();

            foreach (var loc in locations)
            {
                if (!loc.IsOnline)
                    continue;

                var xMinutesAgo = DateTime.UtcNow.AddMinutes(-1 * heartbeatExpiration);
                // Check if last updated time was no later than X minutes ago from now.
                if (loc.LastUpdatedTime >= xMinutesAgo)
                    continue;

                try
                {
                    await _locationRepository.SetCloudOnlineStatusAsync(loc.Id, false);
                }
                catch (Exception ex)
                {
                    Logger.LogError(ex,
                        "There was en error while updating the status of the cloud: {Id}",
                        loc.Id);
                }
            }
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Where was a problem during the execution of {Job} in the cloud",
                nameof(UpdateOnlineStatusJob));
        }
    }
}