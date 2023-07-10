using System.Reflection;
using Middleware.CentralApi.Sdk;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner;
using Middleware.ResourcePlanner.ApiReference;
using Middleware.ResourcePlanner.Config;
using Middleware.ResourcePlanner.Policies;

var builder = WebApplication.CreateBuilder(args);

builder.Configuration
    .AddEnvironmentVariables()
    .AddUserSecrets(Assembly.GetExecutingAssembly(), true);

var centralApiHostname = Environment.GetEnvironmentVariable("CENTRAL_API_HOSTNAME");
if (centralApiHostname is null)
    throw new ArgumentException("Environment variable not defined: CENTRAL_API_HOSTNAME", "CENTRAL_API_HOSTNAME");
var mwConfig = builder.Configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
builder.Services.Configure<MiddlewareConfig>(builder.Configuration.GetSection(MiddlewareConfig.ConfigName));
builder.RegisterSecretsManager();

builder.ConfigureLogger();
// Add services to the container.

builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();
builder.Services.ConfigureAutoMapper();
builder.Services.RegisterCommonServices();
builder.Services.AddHttpClient("healthCheckClient");
builder.Services.AddHttpClient(AppConfig.OrchestratorApiClientName);
builder.Services.AddCentralApiClient(centralApiHostname, mwConfig.Organization);
builder.Services.AddScoped<IResourcePlanner, ResourcePlanner>();
builder.Services.AddScoped<IApiClientBuilder, ApiClientBuilder>();
builder.Services.AddScoped<IPolicyService, PolicyService>();
builder.Services.AddScoped<IPolicyBuilder, PolicyBuilder>();

builder.Services.AddRedisInterfaceClient();

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