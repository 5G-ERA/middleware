using System.Reflection;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.DataAccess.ExtensionMethods;
using Middleware.DataAccess.HostedServices;
using Middleware.RedisInterface;
using Middleware.RedisInterface.Services;
using Middleware.RedisInterface.Services.Abstract;
using Middleware.RedisInterface.Validation;
using Neo4j.Driver;
using Newtonsoft.Json;
using Serilog;

var builder = WebApplication.CreateBuilder(args);

builder.Configuration
    .AddEnvironmentVariables()
    .AddUserSecrets(Assembly.GetExecutingAssembly(), true);

builder.RegisterSecretsManager();

builder.ConfigureLogger();

builder.Services.Configure<MiddlewareConfig>(builder.Configuration.GetSection(MiddlewareConfig.ConfigName));

// Add services to the container.
builder.Services.AddControllers(options =>
    {
        options.InputFormatters.Insert(0, JsonPatchInputFormatter.GetJsonPatchInputFormatter());
    })
    .AddNewtonsoftJson(x => { x.SerializerSettings.ReferenceLoopHandling = ReferenceLoopHandling.Ignore; });
builder.Services.AddFluentValidation(typeof(Program));

// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle

using var driver = GraphDatabase.Driver("bolt://localhost:7687", AuthTokens.Basic("neo4j", "password"));
var queryOperation = await driver.ExecutableQuery("CREATE (n) RETURN n").ExecuteAsync();

builder.Services.AddSwaggerGen();
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddHttpClient("healthCheckClient");

builder.RegisterRedis();
builder.Services.AddUriHelper();
builder.Services.AddHostedService<IndexCreationService>();
builder.Services.RegisterRepositories();
builder.Services.AddScoped<IDashboardService, DashboardService>();
builder.Services.AddScoped<IActionService, ActionService>();
builder.Services.AddScoped<ISliceService, SliceService>();
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