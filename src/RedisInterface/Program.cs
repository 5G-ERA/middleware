using Middleware.Common.ExtensionMethods;
using Middleware.RedisInterface;
using Middleware.RedisInterface.Services;
using Middleware.DataAccess.ExtensionMethods;
using Middleware.RedisInterface.Services.Abstract;
using Middleware.DataAccess.HostedServices;

var builder = WebApplication.CreateBuilder(args);

builder.RegisterSecretsManager();

builder.UseElasticSerilogLogger();


// Add services to the container.
builder.Services.AddControllers(options =>
{
    options.InputFormatters.Insert(0, JsonPatchInputFormatter.GetJsonPatchInputFormatter());
})
.AddNewtonsoftJson(x =>
{
    x.SerializerSettings.ReferenceLoopHandling = Newtonsoft.Json.ReferenceLoopHandling.Ignore;
});

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

//app.UseHttpsRedirection();

app.UseAuthorization();

app.MapControllers();

app.Run();
