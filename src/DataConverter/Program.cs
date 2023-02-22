using System.Reflection;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging.Abstractions;
using Middleware.Common.Config;
using Middleware.DataAccess.Repositories;
using Redis.OM;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

ConfigurationManager configurationManager = new ConfigurationManager();
configurationManager.AddUserSecrets(Assembly.GetAssembly(typeof(Program)));
var redisConfig = configurationManager.GetSection(RedisConfig.ConfigName).Get<RedisConfig>();


ConnectionMultiplexer multiplexer = ConnectionMultiplexer.Connect(
    redisConfig.HostName,
    (c) => { c.Password = redisConfig.Password; });
IRedisGraphClient redisGraphClient = new RedisGraphClient(multiplexer);

//For the redis-cluster master node, port 6380 should be used, using port 6379 will point to the replicas of the redis-cluster
ConnectionMultiplexer clusterMultiplexer =
    ConnectionMultiplexer.Connect($"{redisConfig.ClusterHostname}:6380", c => c.Password = redisConfig.Password);
IRedisConnectionProvider clusterConnectionProvider = new RedisConnectionProvider(clusterMultiplexer);

IRedisGraphClient clusterGraphClient = new RedisGraphClient(clusterMultiplexer);

#region Robot

var robotRepository = new RobotRepository(multiplexer, redisGraphClient, new NullLogger<RobotRepository>());
var clusterRobotRepository =
    new RedisRobotRepository(clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var robots = await robotRepository.GetAllAsync();

foreach (var robot in robots)
{
    await clusterRobotRepository.AddAsync(robot);
}
#endregion


Console.WriteLine("The data conversion process has finished. Press any key to close.");
Console.ReadKey();