using System.Reflection;
using Microsoft.Extensions.Configuration;
using Middleware.Common.Config;
using Middleware.DataAccess.Repositories;
using Redis.OM;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;
using StackExchange.Redis;

var configurationManager = new ConfigurationManager();
configurationManager.AddUserSecrets(Assembly.GetAssembly(typeof(Program)));
var redisConfig = configurationManager.GetSection(RedisConfig.ConfigName).Get<RedisConfig>();


var multiplexer = ConnectionMultiplexer.Connect(
    redisConfig.ClusterHostname,
    c => { c.Password = redisConfig.Password; });
IRedisGraphClient redisGraphClient = new RedisGraphClient(multiplexer);

//For the redis-cluster master node, port 6380 should be used, using port 6379 will point to the replicas of the redis-cluster
var clusterMultiplexer =
    ConnectionMultiplexer.Connect($"{redisConfig.ClusterHostname}:6380", c => c.Password = redisConfig.Password);
IRedisConnectionProvider clusterConnectionProvider = new RedisConnectionProvider(clusterMultiplexer);

IRedisGraphClient clusterGraphClient = new RedisGraphClient(clusterMultiplexer);

var edgeRepository = new RedisEdgeRepository(clusterConnectionProvider, clusterGraphClient, Log.Logger);
var cloudRepository = new RedisCloudRepository(clusterConnectionProvider, clusterGraphClient, Log.Logger);
var locationRepository = new RedisLocationRepository(clusterConnectionProvider, clusterGraphClient, Log.Logger);


var edges = await edgeRepository.GetAllAsync();
foreach (var edge in edges)
{
    var loc = edge.ToLocation();
    await locationRepository.AddAsync(loc);
}

var clouds = await cloudRepository.GetAllAsync();
foreach (var cloud in clouds)
{
    var loc = cloud.ToLocation();
    await locationRepository.AddAsync(loc);
}


Console.WriteLine("The data conversion process has finished. Press any key to close.");
Console.ReadLine();