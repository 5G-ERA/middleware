using System.Net;
using System.Reflection;
using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Sdk;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.MessageContracts;
using Middleware.DataAccess.ExtensionMethods;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.DataAccess.Repositories.Redis;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.ExtensionMethods;
using Middleware.Orchestrator.Heartbeat;
using Middleware.Orchestrator.Installer;
using Middleware.Orchestrator.Publishers;
using Middleware.Orchestrator.SliceManager;
using Middleware.Orchestrator.SliceManager.Contracts;
using Middleware.RedisInterface.Sdk;

var builder = WebApplication.CreateBuilder(args);

builder.Configuration
    .AddEnvironmentVariables()
    .AddUserSecrets(Assembly.GetExecutingAssembly(), true);

builder.RegisterSecretsManager();

builder.ConfigureLogger();
builder.Services.Configure<MiddlewareConfig>(builder.Configuration.GetSection(MiddlewareConfig.ConfigName));
builder.Services.Configure<SliceConfig>(builder.Configuration.GetSection(SliceConfig.ConfigName));
builder.Services.Configure<UserConfig>(builder.Configuration.GetSection(UserConfig.ConfigName));
var mwConfig = builder.Configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();

builder.Services.Configure<InfluxConfig>(builder.Configuration.GetSection(InfluxConfig.ConfigName));
var influxConfig = builder.Configuration.GetSection(InfluxConfig.ConfigName).Get<InfluxConfig>();

var centralApiHostname = Environment.GetEnvironmentVariable("CENTRAL_API_HOSTNAME");
if (centralApiHostname is null)
    throw new ArgumentException("Environment variable not defined: CENTRAL_API_HOSTNAME", "CENTRAL_API_HOSTNAME");
builder.Host.ConfigureAppConfiguration((hostingContext, _) =>
{
    AppConfig.AppConfiguration = hostingContext.HostingEnvironment.EnvironmentName;
    AppConfig.MiddlewareDeploymentLocationName = mwConfig.InstanceName ??
                                                 throw new ArgumentNullException(nameof(mwConfig.InstanceName),
                                                     "Name of the middleware instance has not been specified.");
    ServicePointManager.DnsRefreshTimeout = 60000;
    ServicePointManager.EnableDnsRoundRobin = true;
});
// Add services to the container.
builder.Services.RegisterCommonServices();
builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();

builder.RegisterRedis();
builder.RegisterInflux();

var rabbitmqConfig = builder.Configuration.GetSection(RabbitMqConfig.ConfigName).Get<RabbitMqConfig>();

builder.Services.RegisterRabbitMqConsumers(rabbitmqConfig, mwConfig)
    .ConfigureAutoMapper();
builder.Services.AddHttpClient(AppConfig.OsmApiClientName);
builder.Services.AddScoped<IKubernetesBuilder, KubernetesBuilder>();
builder.Services.AddScoped<IDeploymentService, DeploymentService>();
builder.Services.AddScoped<IRobotStatusRepository, RedisRobotStatusRepository>();
builder.Services.AddScoped<INetAppStatusRepository, RedisNetAppStatusRepository>();
builder.Services.RegisterRepositories();

builder.Services.AddScoped<ISliceManagerClientFactory, SliceManagerClientFactory>();
builder.Services.AddScoped<IKubernetesObjectBuilder, KubernetesObjectBuilder>();
builder.Services.AddScoped<IRosConnectionBuilderFactory, RosConnectionBuilderFactory>();
builder.Services.AddScoped<IPublishingService, PublishingService>();
builder.Services.AddScoped<IPublisher<GatewayAddNetAppEntryMessage>, GatewayAddNetAppEntryPublisher>();
builder.Services.AddScoped<IPublisher<GatewayDeleteNetAppEntryMessage>, GatewayDeleteNetAppEntryPublisher>();
builder.Services.AddScoped<IHeartbeatService, HeartbeatService>();
builder.Services.AddScoped<IStartupDataInstaller, StartupDataInstaller>();

builder.Services.AddRedisInterfaceClient();

builder.Services.AddHttpClient("healthCheckClient");

builder.Services.RegisterQuartzJobs();

builder.Services.AddCentralApiClient(centralApiHostname, mwConfig.Organization);

var app = builder.Build();
using (var scope = app.Services.CreateScope())
{
    var centralApiClient = scope.ServiceProvider.GetService<ICentralApiClient>();
    var request = new RegisterRequest
    {
        Name = mwConfig.InstanceName,
        Organization = mwConfig.Organization,
        Type = mwConfig.InstanceType,
        Address = mwConfig.Address
    };
    var result = await centralApiClient!.RegisterLocation(request);
    if (result is null) throw new("Cannot register at centralAPI");

    AppConfig.MiddlewareId = result.Id;

    var dataInstaller = scope.ServiceProvider.GetService<IStartupDataInstaller>();
    await dataInstaller!.InitializeStartupDataAsync();
}

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