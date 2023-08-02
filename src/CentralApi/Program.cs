using Microsoft.OpenApi.Models;
using Middleware.CentralApi.Auth;
using Middleware.CentralApi.Services;
using Middleware.Common.ExtensionMethods;
using Middleware.DataAccess.ExtensionMethods;
using Middleware.DataAccess.Repositories;
using Middleware.DataAccess.Repositories.Abstract;

var builder = WebApplication.CreateBuilder(args);

// Add services to the container.
builder.RegisterSecretsManager();

builder.ConfigureLogger();
builder.RegisterRedis();
builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen(setup =>
{
    setup.AddSecurityDefinition(ApiKeyAuthOptions.DefaultScheme, new()
    {
        In = ParameterLocation.Header,
        Name = ApiKeyAuthOptions.HeaderName,
        Type = SecuritySchemeType.ApiKey
    });
    setup.AddSecurityRequirement(new()
    {
        {
            new()
            {
                Reference = new()
                {
                    Type = ReferenceType.SecurityScheme,
                    Id = ApiKeyAuthOptions.DefaultScheme
                }
            },
            Array.Empty<string>()
        }
    });
});


builder.Services.AddScoped<ICloudRepository, RedisCloudRepository>();
builder.Services.AddScoped<IEdgeRepository, RedisEdgeRepository>();
builder.Services.AddScoped<IRobotRepository, RedisRobotRepository>();
builder.Services.AddScoped<ILocationService, LocationService>();
builder.Services
    .AddScoped<IApiKeyService, ApiKeyService>()
    .AddScoped<ICacheService, CacheService>()
    .AddScoped<ApiKeyAuthenticationHandler>();

builder.Services.AddAuthentication()
    .AddScheme<ApiKeyAuthOptions, ApiKeyAuthenticationHandler>(ApiKeyAuthOptions.DefaultScheme, null);

var app = builder.Build();

// Configure the HTTP request pipeline.
if (app.Environment.IsDevelopment())
{
    app.UseSwagger();
    app.UseSwaggerUI();
}

//app.UseHttpsRedirection();

//app.UseAuthorization();

app.MapControllers();

app.Run();