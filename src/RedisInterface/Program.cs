using StackExchange.Redis;

var builder = WebApplication.CreateBuilder(args);

// Add services to the container.

builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();

var redisHostname = Environment.GetEnvironmentVariable("REDIS_HOSTNAME") ?? "127.0.0.1";
var redisPort = Environment.GetEnvironmentVariable("REDIS_PORT") ?? "6379";

ConnectionMultiplexer multiplexer = ConnectionMultiplexer.Connect($"{redisHostname}:{redisPort}");
builder.Services.AddSingleton<IConnectionMultiplexer>(multiplexer);
//ConnectionMultiplexer multiplexer = ConnectionMultiplexer.Connect("127.0.0.1:6379");

var app = builder.Build();

// Configure the HTTP request pipeline.
if (app.Environment.IsDevelopment())
{
    app.UseSwagger();
    app.UseSwaggerUI();
}

app.UseHttpsRedirection();

app.UseAuthorization();

app.MapControllers();

app.Run();
