using Middleware.Common.Repositories;
using Middleware.Common.Repositories.Abstract;
using Middleware.RedisInterface;
using RedisGraphDotNet.Client;
using Serilog;
using Serilog.Sinks.Elasticsearch;
using StackExchange.Redis;

var builder = WebApplication.CreateBuilder(args);

#if DEBUG
Serilog.Debugging.SelfLog.Enable(msg => Console.WriteLine(msg));
#endif


Uri elasticsearchUri = new Uri("https://elastic.uri");

Log.Logger = new LoggerConfiguration()
    .Enrich.FromLogContext()
    .Enrich.WithMachineName()
    .Enrich.WithEnvironmentUserName()
    .WriteTo.Console()
    .WriteTo.Elasticsearch(new ElasticsearchSinkOptions(elasticsearchUri)
    {
        ModifyConnectionSettings = conn => conn.BasicAuthentication("user", "pass"),
        ConnectionTimeout = new TimeSpan(0, 1, 0),
        IndexFormat = $"{builder.Configuration["ApplicationName"]}-logs-{builder.Environment.EnvironmentName.ToLower().Replace('.', '-')}-{DateTime.UtcNow:yyyy.MM}",
        AutoRegisterTemplate = true,
        EmitEventFailure = EmitEventFailureHandling.WriteToSelfLog |
                       EmitEventFailureHandling.WriteToFailureSink |
                       EmitEventFailureHandling.RaiseCallback,
        FailureSink = new LoggerConfiguration().WriteTo.Console().CreateLogger()
    })
    .CreateLogger();
builder.Host.UseSerilog(Log.Logger);
builder.Services.AddSingleton<Serilog.ILogger>(Log.Logger);

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

var redisHostname = Environment.GetEnvironmentVariable("REDIS_HOSTNAME") ?? "127.0.0.1";
var redisPort = Environment.GetEnvironmentVariable("REDIS_PORT") ?? "6379";

ConnectionMultiplexer multiplexer = ConnectionMultiplexer.Connect($"{redisHostname}:{redisPort}");
builder.Services.AddSingleton<IConnectionMultiplexer>(multiplexer);
RedisGraphClient redisGraphClient = new RedisGraphClient(redisHostname, int.Parse(redisPort));
builder.Services.AddSingleton<IRedisGraphClient>(redisGraphClient);

builder.Services.AddScoped<IActionRepository, ActionRepository>();
builder.Services.AddScoped<IActionPlanRepository, ActionPlanRepository>();
builder.Services.AddScoped<ICloudRepository, CloudRepository>();
builder.Services.AddScoped<IContainerImageRepository, ContainerImageRepository>();
builder.Services.AddScoped<IEdgeRepository, EdgeRepository>();
builder.Services.AddScoped<IInstanceRepository, InstanceRepository>();
builder.Services.AddScoped<IPolicyRepository, PolicyRepository>();
builder.Services.AddScoped<IRobotRepository, RobotRepository>();
builder.Services.AddScoped<ITaskRepository, TaskRepository>();

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
