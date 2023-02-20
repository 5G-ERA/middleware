using System.Net;
using System.Net.Http.Headers;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Repositories;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.ExtensionMethods;

var builder = WebApplication.CreateBuilder(args);

builder.Configuration.AddEnvironmentVariables();

builder.RegisterSecretsManager();

builder.ConfigureLogger();

var mwConfig = builder.Configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();

builder.Host.ConfigureAppConfiguration((hostingContext, _) =>
{
    AppConfig.AppConfiguration = hostingContext.HostingEnvironment.EnvironmentName;
    AppConfig.MiddlewareDeploymentLocationName = mwConfig.InstanceName ??
                                                 throw new ArgumentNullException(nameof(mwConfig.InstanceName),
                                                     "Name of the middleware instance has not been specified.");
    ServicePointManager.DnsRefreshTimeout = 60000;
    ServicePointManager.EnableDnsRoundRobin = true;
});
builder.Services.AddHttpClient(AppConfig.RedisApiClientName, (a) =>
{
    a.BaseAddress = new Uri(Environment.GetEnvironmentVariable("REDIS_INTERFACE_ADDRESS"));
    a.DefaultRequestHeaders.Accept.Add(
        new MediaTypeWithQualityHeaderValue("application/json"));
});
// Add services to the container.

builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();

builder.RegisterRedis();

var rabbitmqConfig = builder.Configuration.GetSection(RabbitMqConfig.ConfigName).Get<RabbitMqConfig>();

builder.Services.RegisterRabbitMqConsumers(rabbitmqConfig, mwConfig)
    .ConfigureAutoMapper();


builder.Services.AddHttpClient(AppConfig.RedisApiClientName, (a) =>
{
    a.BaseAddress = new Uri(Environment.GetEnvironmentVariable("REDIS_INTERFACE_ADDRESS"));
    a.DefaultRequestHeaders.Accept.Add(
        new MediaTypeWithQualityHeaderValue("application/json"));
});
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