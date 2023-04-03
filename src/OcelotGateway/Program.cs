using System.Configuration;
using System.Text;
using Microsoft.AspNetCore.Authentication.JwtBearer;
using Microsoft.Extensions.Options;
using Microsoft.IdentityModel.Tokens;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.DataAccess.ExtensionMethods;
using Middleware.DataAccess.Repositories;
using Middleware.DataAccess.Repositories.Abstract;
using Ocelot.Cache.CacheManager;
using Ocelot.DependencyInjection;
using Ocelot.Middleware;
using Ocelot.Values;
using Microsoft.IdentityModel;
using static IdentityModel.ClaimComparer;
using Microsoft.IdentityModel.Claims;
using Ocelot.Authentication.Middleware;

var builder = WebApplication.CreateBuilder(args);

builder.RegisterSecretsManager();

builder.ConfigureLogger();

builder.Host.ConfigureAppConfiguration((hostingContext, config) =>
{
    config.AddJsonFile($"ocelot.{hostingContext.HostingEnvironment.EnvironmentName}.json", true, true);
});

builder.Services.AddOcelot()
                .AddCacheManager(settings => settings.WithDictionaryHandle());

var config = builder.Configuration.GetSection(JwtConfig.ConfigName).Get<JwtConfig>();
builder.Services.Configure<JwtConfig>(builder.Configuration.GetSection(JwtConfig.ConfigName));

/*builder.Services.AddAuthentication(
    options =>
    {
        options.DefaultScheme = JwtBearerDefaults.AuthenticationScheme;
        options.DefaultAuthenticateScheme = JwtBearerDefaults.AuthenticationScheme;
        options.DefaultChallengeScheme = JwtBearerDefaults.AuthenticationScheme;
        
    }
    ).AddJwtBearer("Bearer", options =>
    {
        options.TokenValidationParameters = new TokenValidationParameters()
        {
            IssuerSigningKey = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(config.Key)),
            ValidAudience = "redisinterfaceAudience",
            ValidIssuer = "redisinterfaceIssuer",
            ValidateIssuerSigningKey = true,
            ValidateLifetime = true,
            ClockSkew = TimeSpan.Zero,
            RoleClaimType = "Role"
        };
    });*/

builder.Services.AddAuthentication(options =>
{
    options.DefaultScheme = JwtBearerDefaults.AuthenticationScheme;
    options.DefaultAuthenticateScheme = JwtBearerDefaults.AuthenticationScheme;
    options.DefaultChallengeScheme = JwtBearerDefaults.AuthenticationScheme;
}
).AddJwtBearer("Bearer", options =>
{
    options.TokenValidationParameters = new TokenValidationParameters()
    {
        IssuerSigningKey = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(config.Key)),
        //NameClaimType = ClaimTypes.NameIdentifier,
        //NameClaimType = ClaimTypes.Role,
        NameClaimType = IdentityModel.JwtClaimTypes.Name,
        RoleClaimType = IdentityModel.JwtClaimTypes.Role,
        ValidAudience = "redisinterfaceAudience",
        ValidIssuer = "redisinterfaceIssuer",
        ValidateIssuerSigningKey = true,
        ValidateLifetime = true,
        ClockSkew = TimeSpan.Zero
    };
    //builder.Configuration.Bind("JwtSettings", options);
    
});


builder.RegisterRedis();

builder.Services.AddScoped<IUserRepository, RedisUserRepository>();

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
