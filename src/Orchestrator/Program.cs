using System.Net;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.RedisInterface;

var builder = WebApplication.CreateBuilder(args);

// Add services to the container.

builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();
builder.Services.ConfigureAutoMapper();
builder.Services.AddHttpClient(AppConfig.RedisApiClientName);
builder.Services.AddHttpClient(AppConfig.RedisApiClientName);
builder.Services.AddScoped<IApiClientBuilder, ApiClientBuilder>();
var app = builder.Build();

// Configure the HTTP request pipeline.
if (app.Environment.IsDevelopment())
{
    app.UseSwagger();
    app.UseSwaggerUI();
}
ServicePointManager
        .ServerCertificateValidationCallback +=
    (sender, cert, chain, sslPolicyErrors) => true;
app.UseHttpsRedirection();

app.UseAuthorization();

app.MapControllers();

app.Run();