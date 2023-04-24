using DotNet.Testcontainers.Builders;
using DotNet.Testcontainers.Containers;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Mvc.Testing;
using Microsoft.AspNetCore.TestHost;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.DependencyInjection.Extensions;
using Microsoft.Extensions.Logging;
using Middleware.RedisInterface;
using Redis.OM;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace RedisInterface.Sdk.Tests.Integration;

public class RedisInterfaceApiFactory : WebApplicationFactory<IApiMarker>, IAsyncLifetime
{
    private readonly DockerContainer _dbContainer = new ContainerBuilder<DockerContainer>()
        .WithImage("redis/redis-stack")
        .WithPortBinding(6379, 6379)
        .WithWaitStrategy(Wait.ForUnixContainer().UntilPortIsAvailable(6379))
        .Build();
    
    protected override void ConfigureWebHost(IWebHostBuilder builder)
    {
        builder.ConfigureLogging(logging =>
        {
            logging.ClearProviders();
        });
        builder.ConfigureTestServices(services =>
        {
            services.RemoveAll(typeof(IRedisConnectionProvider));
            services.RemoveAll(typeof(IRedisGraphClient));
            
            var mux = ConnectionMultiplexer.Connect($"localhost:6379");
            IRedisConnectionProvider provider = new RedisConnectionProvider(mux);
            services.AddSingleton(provider);
            services.AddSingleton<IConnectionMultiplexer>(mux);
            RedisGraphClient redisGraphClient = new RedisGraphClient(mux);
            services.AddSingleton<IRedisGraphClient>(redisGraphClient);
        });
    }

    public async Task InitializeAsync()
    {
        await _dbContainer.StartAsync();
        
    }

    public new async Task DisposeAsync()
    {
        await _dbContainer.DisposeAsync();
    }
}