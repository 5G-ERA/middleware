using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.RedisInterface;
using Middleware.RedisInterface.Services;
using Middleware.DataAccess.ExtensionMethods;
using Middleware.DataAccess.Repositories;
using Middleware.DataAccess.Repositories.Abstract;
using Redis.OM;
using Middleware.DataAccess.Repositories.Redis;
using Middleware.RedisInterface.Services.Abstract;
using Middleware.DataAccess.HostedServices;

var builder = WebApplication.CreateBuilder(args);

builder.RegisterSecretsManager();

builder.UseElasticSerilogLogger();


// Add services to the container.
builder.Services.AddControllers(options =>
{
    options.InputFormatters.Insert(0, JsonPatchInputFormatter.GetJsonPatchInputFormatter());
})
.AddNewtonsoftJson(x =>
{
    x.SerializerSettings.ReferenceLoopHandling = Newtonsoft.Json.ReferenceLoopHandling.Ignore;
});

// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle

builder.Services.AddSwaggerGen();
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddHttpClient("healthCheckClient");

builder.RegisterRedis();
builder.Services.AddUriHelper();

var provider = new RedisConnectionProvider("redis://localhost:6379");
builder.Services.AddSingleton(provider);
builder.Services.AddScoped<RedisActionRepository, RedisActionRepository>();
builder.Services.AddScoped<IActionService, ActionService>();
builder.Services.AddHostedService<IndexCreationService>();

builder.Services.AddScoped<IActionRepository, ActionRepository>();
builder.Services.AddScoped<IActionPlanRepository, ActionPlanRepository>();
builder.Services.AddScoped<ICloudRepository, CloudRepository>();
builder.Services.AddScoped<IContainerImageRepository, ContainerImageRepository>();
builder.Services.AddScoped<IEdgeRepository, EdgeRepository>();
builder.Services.AddScoped<IInstanceRepository, InstanceRepository>();
builder.Services.AddScoped<IPolicyRepository, PolicyRepository>();
builder.Services.AddScoped<IRobotRepository, RobotRepository>();
builder.Services.AddScoped<ITaskRepository, TaskRepository>();
builder.Services.AddScoped<IDashboardService, DashboardService>();

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
