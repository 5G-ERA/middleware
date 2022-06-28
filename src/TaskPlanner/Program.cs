using Middleware.Common.ExtensionMethods;
using Middleware.TaskPlanner;
using Middleware.TaskPlanner.ApiReference;
using Middleware.TaskPlanner.Config;

var builder = WebApplication.CreateBuilder(args);

builder.RegisterSecretsManager();

builder.UseElasticSerilogLogger();
// Add services to the container.

builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();
builder.Services.ConfigureAutoMapper();

builder.Services.AddHttpClient("healthCheckClient");
builder.Services.AddHttpClient("redisApiClient");
builder.Services.AddHttpClient("resourcePlannerApiClient");
builder.Services.AddHttpClient("orchestratorApiClient");
builder.Services.RegisterCommonServices();
builder.Services.AddScoped<IApiClientBuilder, ApiClientBuilder>();
builder.Services.AddScoped<IActionPlanner, ActionPlanner>();

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
