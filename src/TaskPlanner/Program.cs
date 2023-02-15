using System.Net.Http.Headers;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.MessageContracts;
using Middleware.TaskPlanner.ApiReference;
using Middleware.TaskPlanner.Config;
using Middleware.TaskPlanner.ExtensionMethods;
using Middleware.TaskPlanner.Publishers;
using Middleware.TaskPlanner.Services;

var builder = WebApplication.CreateBuilder(args);
builder.Configuration.AddEnvironmentVariables();

builder.RegisterSecretsManager();

builder.ConfigureLogger();
var mqConfig = builder.Configuration.GetSection(RabbitMqConfig.ConfigName).Get<RabbitMqConfig>();
// Add services to the container.
builder.Services.RegisterRabbitMqPublishers(mqConfig);
builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();
builder.Services.ConfigureAutoMapper();

builder.Services.AddHttpClient("healthCheckClient");
builder.Services.AddHttpClient("resourcePlannerApiClient");
builder.Services.AddHttpClient("orchestratorApiClient");
builder.Services.AddHttpClient(AppConfig.RedisApiClientName, (a) =>
{
    a.BaseAddress = new Uri(Environment.GetEnvironmentVariable("REDIS_INTERFACE_ADDRESS"));
    a.DefaultRequestHeaders.Accept.Add(
        new MediaTypeWithQualityHeaderValue("application/json"));
});

builder.Services.RegisterCommonServices();
builder.Services.AddScoped<IApiClientBuilder, ApiClientBuilder>();
builder.Services.AddScoped<IActionPlanner, ActionPlanner>();
builder.Services.AddScoped<IRedisInterfaceClientService, RedisInterfaceClientService>();
builder.Services.AddScoped<IPublisher<DeployPlanMessage>, DeployPlanMessagePublisher>();

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
