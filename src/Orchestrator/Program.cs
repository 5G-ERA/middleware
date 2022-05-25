using System.Net;
using Middleware.Common.ExtensionMethods;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Jobs;
using Quartz;

var builder = WebApplication.CreateBuilder(args);

builder.RegisterSecretsManager();

builder.UseElasticSerilogLogger();

builder.Host.ConfigureAppConfiguration((hostingContext, config) =>
{
    AppConfig.AppConfiguration = hostingContext.HostingEnvironment.EnvironmentName;
    ServicePointManager.DnsRefreshTimeout = 60000;
    ServicePointManager.EnableDnsRoundRobin = true;
});

// Add services to the container.

builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();

builder.Services.ConfigureAutoMapper();
builder.Services.AddHttpClient(AppConfig.RedisApiClientName);
builder.Services.AddHttpClient(AppConfig.OsmApiClientName);
builder.Services.RegisterCommonServices();
builder.Services.AddScoped<IApiClientBuilder, ApiClientBuilder>();
builder.Services.AddScoped<IKubernetesBuilder, KubernetesBuilder>();
builder.Services.AddScoped<IDeploymentService, DeploymentService>();

builder.Services.AddHttpClient("healthCheckClient");

// quartz 
builder.Services.AddQuartz(q =>
{
    q.UseMicrosoftDependencyInjectionJobFactory();

    q.ScheduleJob<MiddlewareStartupJob>(trg => trg
        .WithIdentity("Middleware startup Job")
        .WithDescription("Job that starts the whole Middleware system")
        .StartNow()
    );
});
builder.Services.AddQuartzHostedService(opt =>
{
    opt.WaitForJobsToComplete = true;
});

builder.Services.AddTransient<MiddlewareStartupJob>();

var app = builder.Build();

// Configure the HTTP request pipeline.
if (app.Environment.IsDevelopment())
{
    app.UseSwagger();
    app.UseSwaggerUI();
}

//app.UseHttpsRedirection();

app.UseAuthorization();

app.MapControllers();

app.Run();