using Ocelot.DependencyInjection;
using Ocelot.Middleware;
using Ocelot.Cache.CacheManager;
using Microsoft.IdentityModel.Tokens;
using System.Text;
using Microsoft.AspNetCore.Authentication.JwtBearer;
using Middleware.Common.Repositories.Abstract;
using Middleware.Common.Repositories;
using StackExchange.Redis;
using RedisGraphDotNet.Client;
using Serilog;


var builder = WebApplication.CreateBuilder(args);

/*builder.Host.ConfigureLogging((hostingContext, loggingbuilder) =>
{
    loggingbuilder.AddConfiguration(hostingContext.Configuration.GetSection("Logging"));
    loggingbuilder.AddConsole();
    loggingbuilder.AddDebug();
});*/

builder.Host.UseSerilog((ctx, lc) => lc
    .MinimumLevel.Debug()
    .WriteTo.Console());

builder.Host.ConfigureAppConfiguration((hostingContext, config) =>
{
    config.AddJsonFile($"ocelot.{hostingContext.HostingEnvironment.EnvironmentName}.json", true, true);
});

builder.Services.AddOcelot()
                .AddCacheManager(settings => settings.WithDictionaryHandle());

builder.Services.AddAuthentication(
    options =>
    {
        options.DefaultAuthenticateScheme = JwtBearerDefaults.AuthenticationScheme;
        options.DefaultChallengeScheme = JwtBearerDefaults.AuthenticationScheme;
    }
    ).AddJwtBearer("Bearer", options =>
    {
        options.TokenValidationParameters = new TokenValidationParameters()
        {
            IssuerSigningKey = new SymmetricSecurityKey(Encoding.UTF8.GetBytes("my_secure_api_secret")),
            ValidAudience = "redisinterfaceAudience",
            ValidIssuer = "redisinterfaceIssuer",
            ValidateIssuerSigningKey = true,
            ValidateLifetime = true,
            ClockSkew = TimeSpan.Zero
        };
    });

var redisHostname = Environment.GetEnvironmentVariable("REDIS_HOSTNAME") ?? "127.0.0.1";
var redisPort = Environment.GetEnvironmentVariable("REDIS_PORT") ?? "6379";

ConnectionMultiplexer multiplexer = ConnectionMultiplexer.Connect($"{redisHostname}:{redisPort}");
builder.Services.AddSingleton<IConnectionMultiplexer>(multiplexer);
RedisGraphClient redisGraphClient = new RedisGraphClient(redisHostname, int.Parse(redisPort));
builder.Services.AddSingleton<IRedisGraphClient>(redisGraphClient);

builder.Services.AddScoped<IUserRepository, UserRepository>();

var app = builder.Build();

app.UseRouting();
app.UseAuthentication();
app.UseAuthorization();
app.UseEndpoints(endpoints =>
{
    endpoints.MapControllers();
});

await app.UseOcelot();

app.MapGet("/", () => "OcelotGateway is functional!");

app.Run();
