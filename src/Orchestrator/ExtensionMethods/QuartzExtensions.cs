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
                .StartAt(DateBuilder.EvenMinuteDateBefore(DateTimeOffset.UtcNow.AddMinutes(2))));

            q.ScheduleJob<RefreshMiddlewareAddressJob>(trg => trg
                .WithIdentity("Update the Middleware address Job")
                .WithDescription("Updates the current address under which the Middleware has to be contacted")
                .WithSimpleSchedule(x => x.WithInterval(TimeSpan.FromMinutes(1)).RepeatForever())
                .StartAt(DateBuilder.EvenMinuteDateBefore(DateTimeOffset.UtcNow.AddMinutes(1))));

        });
        services.AddQuartzHostedService(opt =>
        {
            opt.WaitForJobsToComplete = true;
        });
        services.AddTransient<MiddlewareStartupJob>();
        services.AddTransient<UpdateStatusJob>();
        services.AddTransient<RefreshMiddlewareAddressJob>();

        return services;
    }
}
