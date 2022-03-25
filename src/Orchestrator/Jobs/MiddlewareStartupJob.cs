using Quartz;

namespace Middleware.Orchestrator.Jobs
{
    public class MiddlewareStartupJob : IJob
    {
        private readonly ILogger<MiddlewareStartupJob> _logger;

        public MiddlewareStartupJob(ILogger<MiddlewareStartupJob> logger)
        {
            _logger = logger;
        }
        public async Task Execute(IJobExecutionContext context)
        {
            _logger.LogInformation("Executed");
            //throw new NotImplementedException();
        }
    }
}
