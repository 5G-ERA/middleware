using FluentValidation;
using FluentValidation.AspNetCore;
using Middleware.Common.ExtensionMethods;
using Middleware.RedisInterface;
using Middleware.RedisInterface.Services;
using Middleware.DataAccess.ExtensionMethods;
using Middleware.RedisInterface.Services.Abstract;
using Middleware.DataAccess.HostedServices;
using Middleware.RedisInterface.Validation;
using Serilog;

var builder = WebApplication.CreateBuilder(args);


builder.RegisterSecretsManager();

builder.ConfigureLogger();


// Add services to the container.
builder.Services.AddControllers(options =>
{
    options.InputFormatters.Insert(0, JsonPatchInputFormatter.GetJsonPatchInputFormatter());
})
.AddNewtonsoftJson(x =>
{
    x.SerializerSettings.ReferenceLoopHandling = Newtonsoft.Json.ReferenceLoopHandling.Ignore;
});
builder.Services.AddFluentValidation(typeof(Program));

// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle

builder.Services.AddSwaggerGen();
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddHttpClient("healthCheckClient");

builder.RegisterRedis();
builder.Services.AddUriHelper();
builder.Services.AddHostedService<IndexCreationService>();
builder.Services.RegisterRepositories();
builder.Services.AddScoped<IDashboardService, DashboardService>();
builder.Services.AddScoped<IActionService, ActionService>();
var app = builder.Build();

// Configure the HTTP request pipeline.
if (app.Environment.IsDevelopment())
{
    app.UseSwagger();
    app.UseSwaggerUI();
}
app.UseSerilogRequestLogging();

app.UseMiddleware<ValidationExceptionMiddleware>();
//app.UseHttpsRedirection();

app.UseAuthorization();

app.MapControllers();

app.Run();
