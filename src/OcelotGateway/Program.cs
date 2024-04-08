using System.Text;
using IdentityModel;
using Microsoft.AspNetCore.Authentication.JwtBearer;
using Microsoft.IdentityModel.Tokens;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.DataAccess.ExtensionMethods;
using Middleware.DataAccess.Repositories;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.OcelotGateway.ExtensionMethods;
using Middleware.OcelotGateway.Services;
using Ocelot.Cache.CacheManager;
using Ocelot.DependencyInjection;
using Ocelot.Middleware;
using Yarp.ReverseProxy.Configuration;

var builder = WebApplication.CreateBuilder(args);

builder.RegisterSecretsManager();

builder.ConfigureLogger();

builder.Host.ConfigureAppConfiguration((hostingContext, config) =>
{
    config.AddJsonFile($"ocelot.{hostingContext.HostingEnvironment.EnvironmentName}.json", true, true);
});

builder.Services.AddOcelot()
    .AddCacheManager(settings => settings.WithDictionaryHandle());
builder.Services.DecorateClaimAuthoriser();

var config = builder.Configuration.GetSection(JwtConfig.ConfigName).Get<JwtConfig>();
builder.Services.Configure<JwtConfig>(builder.Configuration.GetSection(JwtConfig.ConfigName));
//var mwConfig = builder.Configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
builder.Services.Configure<MiddlewareConfig>(builder.Configuration.GetSection(MiddlewareConfig.ConfigName));

builder.Services.AddAuthentication(options =>
    {
        options.DefaultScheme = JwtBearerDefaults.AuthenticationScheme;
        options.DefaultAuthenticateScheme = JwtBearerDefaults.AuthenticationScheme;
        options.DefaultChallengeScheme = JwtBearerDefaults.AuthenticationScheme;
    }
).AddJwtBearer("Bearer", options =>
{
    options.TokenValidationParameters = new()
    {
        IssuerSigningKey = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(string.IsNullOrEmpty(config.Key)? "vub21ZNuFLJL2W1qjNOiiIqsL051BW82Xp4dLuRw777" : config.Key)),
        NameClaimType = JwtClaimTypes.Name,
        RoleClaimType = JwtClaimTypes.Role,
        ValidAudience = "redisinterfaceAudience",
        ValidIssuer = "redisinterfaceIssuer",
        ValidateIssuerSigningKey = true,
        ValidateLifetime = true,
        ClockSkew = TimeSpan.Zero
    };
});
builder.RegisterRedis();

builder.Services.AddScoped<IUserRepository, RedisUserRepository>();
builder.Services.AddScoped<GatewayConfigurationService>();

var ocelotConfig = new OcelotPipelineConfiguration
{
    AuthorizationMiddleware = async (httpContext, next) =>
    {
        await OcelotAuthorizationMiddleware.Authorize(httpContext, next);
    }
};
builder.Services.Configure<MiddlewareConfig>(builder.Configuration.GetSection(MiddlewareConfig.ConfigName));
var mwConfig = builder.Configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
var rabbitmqConfig = builder.Configuration.GetSection(RabbitMqConfig.ConfigName).Get<RabbitMqConfig>();

builder.Services.AddReverseProxy()
    .LoadFromMemory(new List<RouteConfig>(), new List<ClusterConfig>());
builder.Services.RegisterRabbitMqConsumers(rabbitmqConfig, mwConfig);


var app = builder.Build();

app.MapReverseProxy();

app.UseRouting();
app.UseAuthentication();
app.UseAuthorization();

app.UseEndpoints(endpoints =>
{
    endpoints.MapControllers();
    //endpoints.MapReverseProxy();
});

await app.UseOcelot(ocelotConfig);

app.MapGet("/", () => "Welcome to 5G-ERA Middleware");

app.Run();