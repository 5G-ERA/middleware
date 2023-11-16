using Microsoft.Extensions.Options;
using Middleware.CentralApi.Sdk;
using Middleware.Common.Config;
using Middleware.Common.Job;
using Middleware.Models.Domain;
using Quartz;

namespace Middleware.Orchestrator.Jobs;

[DisallowConcurrentExecution]
public class UpdateOnlineStatusMiddleware : BaseJob<UpdateOnlineStatusMiddleware>
{
    private readonly IOptions<MiddlewareConfig> _mwConfig;
    private readonly ICentralApiClient _centralApi;

    public UpdateOnlineStatusMiddleware(ICentralApiClient centralApi, IOptions<MiddlewareConfig> mwConfig, ILogger<UpdateOnlineStatusMiddleware> logger) : base(logger)
    {
        _centralApi = centralApi;
        _mwConfig = mwConfig;
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        var middlewareId = AppConfig.MiddlewareId;
        var locationType = _mwConfig.Value.InstanceType;

        if (locationType == null )
        {
            Logger.LogInformation("Could not get Middleware location type.");
            return;
        }
        if(middlewareId == Guid.Empty)
        {
            Logger.LogInformation("Could not get Middleware ID.");
            return;
        }

        var middleware = new CloudEdgeStatusRequest();

        middleware.IsOnline = true;
        middleware.Type = locationType;
        try
        {
            await _centralApi.SetStatus(middleware, middlewareId);
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "There was en error while updating the online status of the Middleware");
        }
    }
}
