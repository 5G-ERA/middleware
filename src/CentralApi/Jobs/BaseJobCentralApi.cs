using Quartz;

namespace Middleware.CentralApi.Jobs;

public abstract class BaseJobCentralApi<T> : IJob
{
    protected readonly ILogger<T> Logger;

    protected BaseJobCentralApi(ILogger<T> logger)
    {
        Logger = logger;
    }

    public async Task Execute(IJobExecutionContext context)
    {
        try
        {
            await ExecuteJobAsync(context);
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Something went wrong during the execution of the action");
        }
    }

    protected abstract Task ExecuteJobAsync(IJobExecutionContext context);
}