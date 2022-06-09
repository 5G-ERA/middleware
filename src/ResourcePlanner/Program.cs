using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.ResourcePlanner;
using Middleware.ResourcePlanner.ApiReference;
using Middleware.ResourcePlanner.Config;

var builder = WebApplication.CreateBuilder(args);

builder.RegisterSecretsManager();

builder.UseElasticSerilogLogger();
// Add services to the container.

builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();
builder.Services.ConfigureAutoMapper();
builder.Services.RegisterCommonServices();
builder.Services.AddHttpClient("healthCheckClient");
builder.Services.AddHttpClient(AppConfig.OrchestratorApiClientName);
builder.Services.AddScoped<IResourcePlanner, ResourcePlanner>();
builder.Services.AddScoped<IApiClientBuilder, ApiClientBuilder>();

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
