﻿using Quartz;

namespace Middleware.Orchestrator.Jobs
{
    public abstract class BaseJob<T> : IJob
    {
        protected readonly ILogger<T> Logger;

        protected BaseJob(ILogger<T> logger)
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
                Logger.LogError("Something went wrong during the execution of the action", ex);
            }
        }

        protected abstract Task ExecuteJobAsync(IJobExecutionContext context);
    }
}