using System.Reflection;
using Middleware.CentralApi.Sdk;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Validation;
using Middleware.DataAccess.ExtensionMethods;
using Middleware.DataAccess.HostedServices;
using Middleware.RedisInterface;
using Middleware.RedisInterface.Services;
using Middleware.RedisInterface.Services.Abstract;
using Newtonsoft.Json;
using Serilog;

var builder = WebApplication.CreateBuilder(args);

builder.Configuration
    .AddEnvironmentVariables()
    .AddUserSecrets(Assembly.GetExecutingAssembly(), true);

builder.RegisterSecretsManager();

builder.ConfigureLogger();

var mwConfig = builder.Configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
var centralApiHostname = Environment.GetEnvironmentVariable("CENTRAL_API_HOSTNAME");
if (centralApiHostname is null)
    throw new ArgumentException("Environment variable not defined: CENTRAL_API_HOSTNAME", "CENTRAL_API_HOSTNAME");

builder.Services.Configure<MiddlewareConfig>(builder.Configuration.GetSection(MiddlewareConfig.ConfigName));

// Add services to the container.
builder.Services.AddControllers(options =>
    {
        options.InputFormatters.Insert(0, JsonPatchInputFormatter.GetJsonPatchInputFormatter());
    })
    .AddNewtonsoftJson(x => { x.SerializerSettings.ReferenceLoopHandling = ReferenceLoopHandling.Ignore; });
builder.Services.AddFluentValidation(typeof(Program));

builder.Services.AddSwaggerGen();
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddHttpClient("healthCheckClient");

builder.RegisterRedis();
builder.Services.AddUriHelper();
builder.Services.AddHostedService<IndexCreationService>();
builder.Services.RegisterRepositories();
builder.Services.AddCentralApiClient(centralApiHostname, mwConfig.Organization);
builder.Services.AddScoped<IDashboardService, DashboardService>();
builder.Services.AddScoped<IActionService, ActionService>();
builder.Services.AddScoped<ISliceService, SliceService>();
builder.Services.AddScoped<ITaskService, TaskService>();
builder.Services.AddScoped<ISystemConfigService, SystemConfigService>();

var app = builder.Build();

// Configure the HTTP request pipeline.
if (app.Environment.IsDevelopment())
{
    app.UseSwagger();
    app.UseSwaggerUI();
}

app.UseSerilogRequestLogging();

app.UseMiddleware<ValidationExceptionMiddleware>();
//app.UseHttpsRedirection();

//app.UseAuthentication();
//app.UseAuthorization();

app.MapControllers();

app.Run();