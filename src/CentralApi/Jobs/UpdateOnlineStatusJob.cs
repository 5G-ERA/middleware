using Middleware.Common.Job;
using Middleware.DataAccess.Repositories.Abstract;
using Quartz;

namespace Middleware.CentralApi.Jobs;

[DisallowConcurrentExecution]
public class UpdateOnlineStatusJob : BaseJob<UpdateOnlineStatusJob>
{
    private readonly ILocationRepository _locationRepository;

    public UpdateOnlineStatusJob(ILogger<UpdateOnlineStatusJob> logger, ILocationRepository locationRepository) :
        base(logger)
    {
        _locationRepository = locationRepository;
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        try
        {
            var locations = await _locationRepository.GetAllAsync();

            foreach (var loc in locations)
            {
                if (!loc.IsOnline)
                    continue;

                var threeMinutesEarlier = DateTime.UtcNow.AddMinutes(-3);
                // Check if last updated time was no later then 3 minutes ago from now.
                if (loc.LastUpdatedTime < threeMinutesEarlier)
                {
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
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Where was a problem during the execution of {Job} in the cloud",
                nameof(UpdateOnlineStatusJob));
        }
    }
}