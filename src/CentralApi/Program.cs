using Middleware.CentralApi.ExtensionMethods;
using Middleware.CentralApi.Services;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Validation;
using Middleware.DataAccess.ExtensionMethods;
using Middleware.CentralApi.Services.Abstract;
using Middleware.DataAccess.HostedServices;

var builder = WebApplication.CreateBuilder(args);

// Add services to the container.
builder.RegisterSecretsManager();

builder.ConfigureLogger();
builder.RegisterRedis();
builder.Services.AddControllers();
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle

builder.Services.AddFluentValidation(typeof(Program));
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();

if (builder.Environment.IsDevelopment() == false)
    builder.Services.AddHostedService<IndexCreationService>();


builder.Services.RegisterCentralApiRepositories();
builder.Services.AddScoped<IRobotService, RobotService>();

builder.Services.AddScoped<ILocationService, LocationService>();

builder.Services.RegisterCentralApiQuartzJobs();

var app = builder.Build();

// Configure the HTTP request pipeline.
if (app.Environment.IsDevelopment())
{
    app.UseSwagger();
    app.UseSwaggerUI();
}

//app.UseHttpsRedirection();

//app.UseAuthorization();

app.UseMiddleware<ValidationExceptionMiddleware>();

app.MapControllers();

app.Run();