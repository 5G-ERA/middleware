using System.Reflection;
using Middleware.CentralApi.Sdk;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.MessageContracts;
using Middleware.RedisInterface.Sdk;
using Middleware.TaskPlanner.ApiReference;
using Middleware.TaskPlanner.Config;
using Middleware.TaskPlanner.ExtensionMethods;
using Middleware.TaskPlanner.Publishers;
using Middleware.TaskPlanner.Services;

var builder = WebApplication.CreateBuilder(args);
var centralApiHostname = Environment.GetEnvironmentVariable("CENTRAL_API_HOSTNAME");
if (centralApiHostname is null)
    throw new ArgumentException("Environment variable not defined: CENTRAL_API_HOSTNAME", "CENTRAL_API_HOSTNAME");

builder.Configuration
    .AddEnvironmentVariables()
    .AddUserSecrets(Assembly.GetExecutingAssembly(), true);
builder.RegisterSecretsManager();

builder.ConfigureLogger();
var mqConfig = builder.Configuration.GetSection(RabbitMqConfig.ConfigName).Get<RabbitMqConfig>();
var mwConfig = builder.Configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
builder.Services.Configure<MiddlewareConfig>(builder.Configuration.GetSection(MiddlewareConfig.ConfigName));
// Add services to the container.
builder.Services.RegisterRabbitMqPublishers(mqConfig, mwConfig);
builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen(s => { s.SupportNonNullableReferenceTypes(); });
builder.Services.ConfigureAutoMapper();

builder.Services.AddHttpClient("healthCheckClient");
builder.Services.AddHttpClient("resourcePlannerApiClient");
builder.Services.AddHttpClient("orchestratorApiClient");
builder.Services.AddCentralApiClient(centralApiHostname, mwConfig.Organization);
builder.Services.RegisterCommonServices();
builder.Services.AddRedisInterfaceClient();
builder.Services.AddScoped<IApiClientBuilder, ApiClientBuilder>();
builder.Services.AddScoped<IActionPlanner, ActionPlanner>();
builder.Services.AddScoped<IPublishService, PublishingService>();
builder.Services.AddScoped<IPublisher<DeployPlanMessage>, DeployPlanMessagePublisher>();
builder.Services.AddScoped<IPublisher<SwitchoverDeleteAction>, SwitchoverDeleteInstancePublisher>();
builder.Services.AddScoped<IPublisher<SwitchoverDeployAction>, SwitchoverDeployInstancePublisher>();
builder.Services.AddScoped<IPublisher<ConnectRobotToSliceMessage>, ConnectRobotToSlicePublisher>();
builder.Services
    .AddScoped<IRequestResponseClient<RequestResourcePlanMessage, RequestResourcePlanMessage>,
        ResourcePlanRequestPublisher>();

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