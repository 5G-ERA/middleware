using System.Net;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Repositories;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.ExtensionMethods;

var builder = WebApplication.CreateBuilder(args);

builder.RegisterSecretsManager();

builder.UseElasticSerilogLogger();

builder.Host.ConfigureAppConfiguration((hostingContext, _) =>
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

builder.RegisterRedis();

builder.Services.ConfigureAutoMapper();
builder.Services.AddHttpClient(AppConfig.RedisApiClientName);
builder.Services.AddHttpClient(AppConfig.OsmApiClientName);
builder.Services.RegisterCommonServices();
builder.Services.AddScoped<IApiClientBuilder, ApiClientBuilder>();
builder.Services.AddScoped<IKubernetesBuilder, KubernetesBuilder>();
builder.Services.AddScoped<IDeploymentService, DeploymentService>();
builder.Services.AddScoped<INetAppStatusRepository, NetAppStatusRepository>();
builder.Services.AddScoped<IRobotStatusRepository, RobotStatusRepository>();

builder.Services.AddHttpClient("healthCheckClient");

builder.Services.RegisterQuartzJobs();

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