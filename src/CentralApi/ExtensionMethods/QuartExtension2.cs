using Middleware.CentralApi.Jobs;
using Middleware.Common.Config;
using Quartz;

namespace Middleware.CentralApi.ExtensionMethods;
public static class QuartExtension2
{    /// <summary>
     ///     Registers the CentralApi Quartz jobs
     /// </summary>
     /// <param name="services"></param>
     /// <returns></returns>
    public static IServiceCollection RegisterCentralApiQuartzJobs(this IServiceCollection services)
    {
        // quartz 
        services.AddQuartz(q =>
        {
            q.UseMicrosoftDependencyInjectionJobFactory();

            q.ScheduleJob<UpdateOnlineStatusJob>(trg => trg
                .WithIdentity("Update online status Job")
                .WithDescription("Set status ofline of cloud and edge if is inactive for three minutes.")
                .WithSimpleSchedule(x => x.WithIntervalInMinutes(1).RepeatForever())
                .StartAt(DateBuilder.EvenMinuteDateBefore(DateTimeOffset.Now.AddMinutes(2)))
                );
            
            q.ScheduleJob<UpdateOnlineLocationJob>(trg => trg
                .WithIdentity("Delete robot-location relation Job")
                .WithDescription("Job that monitors the relation status of the robot-location. " +
                                 "Delete their inactive relations and reports any seen failures.")
                .WithSimpleSchedule(x => x.WithIntervalInMinutes(1).RepeatForever())
                .StartAt(DateBuilder.EvenMinuteDateBefore(DateTimeOffset.Now.AddMinutes(2))));

        });
        services.AddQuartzHostedService(opt => { opt.WaitForJobsToComplete = true; });
        services.AddTransient<UpdateOnlineStatusJob>();
        services.AddTransient<UpdateOnlineLocationJob>();

        return services;
    }
}
