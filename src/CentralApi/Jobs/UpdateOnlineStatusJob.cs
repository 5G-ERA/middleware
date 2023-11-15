using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Quartz;

namespace Middleware.CentralApi.Jobs;

[DisallowConcurrentExecution]
public class UpdateOnlineStatusJob : BaseJobCentralApi<UpdateOnlineStatusJob>
{

    private readonly ILogger _logger;
    private readonly ICloudRepository _cloudRepository;
    private readonly IEdgeRepository _edgeRepository;

    public UpdateOnlineStatusJob(ILogger<UpdateOnlineStatusJob> logger, ICloudRepository cloudRepository, IEdgeRepository edgeRepository) : base(logger)
    {
        _logger = logger;
        _cloudRepository = cloudRepository;
        _edgeRepository = edgeRepository;
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        try
        {
            List<CloudModel> cloudModels = await _cloudRepository.GetAllAsync();

            foreach (CloudModel model in cloudModels)
            {
                bool isOnline = model.IsOnline;
                if (isOnline)
                {
                    DateTime threeMinutesEarlier = DateTime.UtcNow.AddMinutes(-3);
                    // Check if last updated time was no later then 3 minutes ago from now.
                    if (model.LastUpdatedTime < threeMinutesEarlier)
                    {
                        try
                        {
                            await _cloudRepository.SetCloudOnlineStatusAsync(model.Id, false);
                        }
                        catch (Exception ex)
                        {

                            Logger.LogError(ex,
                                "There was en error while updating the status of the cloud: {Id}",
                                model.Id);
                        }
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Where was a problem during the execution of {Job} in the cloud", nameof(UpdateOnlineStatusJob));
            // throw;
        }

        try
        {
            List<EdgeModel> edgeModels = await _edgeRepository.GetAllAsync();

            foreach (EdgeModel model in edgeModels)
            {
                bool isOnline = model.IsOnline;
                if (isOnline)
                {
                    DateTime threeMinutesEarlier = DateTime.UtcNow.AddMinutes(-3);

                    // Check if last updated time was no later then 3 minutes ago from now.
                    if (model.LastUpdatedTime < threeMinutesEarlier)
                    {
                        try
                        {
                            await _edgeRepository.SetEdgeOnlineStatusAsync(model.Id, false);
                        }
                        catch (Exception ex)
                        {
                            Logger.LogError(ex,
                                "There was en error while updating the status of the edge: {Id}",
                                model.Id);
                        }
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Where was a problem during the execution of {Job} in the edge", nameof(UpdateOnlineStatusJob));
            // throw;
        }
    }
}
