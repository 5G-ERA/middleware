using Elasticsearch.Net;
using Elasticsearch.Net.Aws;
using Middleware.RedisInterface;
using Middleware.Common.Repositories;
using Middleware.Common.Repositories.Abstract;
using RedisGraphDotNet.Client;
using Serilog;
using Serilog.Formatting.Json;
using Serilog.Sinks.Elasticsearch;
using StackExchange.Redis;

var builder = WebApplication.CreateBuilder(args);


Log.Logger = new LoggerConfiguration()
    .Enrich.FromLogContext()
    .Enrich.WithExceptionDetails()
    .WriteTo.Elasticsearch(new ElasticsearchSinkOptions(new Uri(awsSettings.ElasticSearchUrl))
    {
        ModifyConnectionSettings = conn =>
        {
            var httpConnection = new AwsHttpConnection(awsSettings.Region
                , new StaticCredentialsProvider(new AwsCredentials
                {
                    AccessKey = awsSettings.AccessKey,
                    SecretKey = awsSettings.SecretKey,
                }));
            var pool = new SingleNodeConnectionPool(new Uri(awsSettings.ElasticSearchUrl));
            var conf = new ConnectionConfiguration(pool, httpConnection);
            return conf;
        },
        ConnectionTimeout = new TimeSpan(0, 10, 0),
        IndexFormat = "xxxxx-{0:yyyy.MM}",
        FailureCallback = e => Console.WriteLine("Unable to submit event " + e.MessageTemplate),
        EmitEventFailure = EmitEventFailureHandling.WriteToSelfLog |
                           EmitEventFailureHandling.WriteToFailureSink |
                           EmitEventFailureHandling.RaiseCallback,
        FailureSink = new LoggerConfiguration().WriteTo
            .RollingFile(new JsonFormatter(), "XXXXX-{Date}.txt").CreateLogger()
    })
    .write
    .CreateLogger();


builder.Host.UseSerilog((ctx, lc) => lc
    .MinimumLevel.Debug()
    .WriteTo.Console());

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
