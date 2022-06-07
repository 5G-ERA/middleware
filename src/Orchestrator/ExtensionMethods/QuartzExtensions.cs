using Middleware.Common.Config;
using Middleware.Orchestrator.Jobs;
using Quartz;

namespace Middleware.Orchestrator.ExtensionMethods;

public static class QuartzExtensions
{
    /// <summary>
    /// Registers the Orchestrator Quartz jobs
    /// </summary>
    /// <param name="services"></param>
    /// <returns></returns>
    public static IServiceCollection RegisterQuartzJobs(this IServiceCollection services)
    {
        // quartz 
        services.AddQuartz(q =>
        {
            q.UseMicrosoftDependencyInjectionJobFactory();

            q.ScheduleJob<MiddlewareStartupJob>(trg => trg
                .WithIdentity("Middleware startup Job")
                .WithDescription("Job that starts the whole Middleware system")
                .StartNow()
            );

            q.ScheduleJob<UpdateStatusJob>(trg => trg
                .WithIdentity("Update service status Job")
                .WithDescription("Job that monitors the status of the deployed services by the Middleware. " +
                                 "Updates their status and reports any seen failures.")
                .WithSimpleSchedule(x => 
                    x.WithInterval(TimeSpan.FromSeconds(AppConfig.StatusCheckInterval)).RepeatForever())
                .StartAt(DateBuilder.EvenMinuteDate(DateTimeOffset.Now.AddMinutes(2))));
            
        });
        services.AddQuartzHostedService(opt =>
        {
            opt.WaitForJobsToComplete = true;
        });
        services.AddTransient<MiddlewareStartupJob>();
        services.AddTransient<UpdateStatusJob>();

        return services;
    }
}