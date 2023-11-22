using Middleware.CentralApi.Jobs;
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
        });
        services.AddQuartzHostedService(opt => { opt.WaitForJobsToComplete = true; });
        services.AddTransient<UpdateOnlineStatusJob>();

        return services;
    }
}
