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
builder.Services.AddSwaggerGen();
builder.Services.AddScoped<ICloudRepository, RedisCloudRepository>();
builder.Services.AddScoped<IEdgeRepository, RedisEdgeRepository>();
builder.Services.AddScoped<ILocationService, LocationService>();


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