using System.Reflection;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging.Abstractions;
using Middleware.Common.Config;
using Middleware.DataAccess.Repositories;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.DataAccess.Repositories.Redis;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Services;
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
Console.WriteLine("Writing Robots...");
var robotRepository = new RobotRepository(multiplexer, redisGraphClient, new NullLogger<RobotRepository>());
var clusterRobotRepository =
    new RedisRobotRepository(clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var robots = await robotRepository.GetAllAsync();

foreach (var robot in robots)
{
    await clusterRobotRepository.AddAsync(robot);
}
#endregion

#region action
Console.WriteLine("Writing Actions...");
var actionRepository = new ActionRepository(multiplexer, redisGraphClient, new NullLogger<ActionRepository>());
var clusterActionRepository =
    new RedisActionRepository(clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var actions = await actionRepository.GetAllAsync();

foreach (var action in actions)
{
    await clusterActionRepository.AddAsync(action);
}

#endregion


#region cloud
Console.WriteLine("Writing Clouds...");
var cloudRepository = new CloudRepository(multiplexer, redisGraphClient, new NullLogger<CloudRepository>());
var redisCloudRepository =
    new RedisCloudRepository(clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var clouds = await cloudRepository.GetAllAsync();

foreach (var cloud in clouds)
{
    await redisCloudRepository.AddAsync(cloud);
}

#endregion


#region instance
Console.WriteLine("Writing Instances...");
var instanceRepository = new InstanceRepository(multiplexer, redisGraphClient, new NullLogger<InstanceRepository>());
var redisInstanceRepository =
    new RedisInstanceRepository(clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var instances = await instanceRepository.GetAllAsync();

foreach (var instance in instances)
{
    await redisInstanceRepository.AddAsync(instance);
}

#endregion

#region continerImage
Console.WriteLine("Writing Container images...");
var containerImageRepository = new ContainerImageRepository(instanceRepository, multiplexer, redisGraphClient, new NullLogger<ContainerImageRepository>());
var redisContainerImageRepository =
    new RedisContainerImageRepository(redisInstanceRepository, clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var containers = await containerImageRepository.GetAllAsync();

foreach (var container in containers)
{
    await redisContainerImageRepository.AddAsync(container);
}

#endregion

#region Edge
Console.WriteLine("Writing Edges...");
var edgeRepository = new EdgeRepository(multiplexer, redisGraphClient, new NullLogger<EdgeRepository>());
var redisEdgeRepository =
    new RedisEdgeRepository(clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var edges = await edgeRepository.GetAllAsync();

foreach (var edge in edges)
{
    await redisEdgeRepository.AddAsync(edge);
}

#endregion


#region Users
Console.WriteLine("Writing Users...");
var userRepository = new UserRepository(multiplexer, redisGraphClient, new NullLogger<UserRepository>());
var redisUserRepository =
    new RedisUserRepository(clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var users = await userRepository.GetAllAsync();

foreach (var user in users)
{
    await redisUserRepository.AddAsync(user);
}

#endregion


#region Policy
Console.WriteLine("Writing Policies...");
var policyRepository = new PolicyRepository(multiplexer, redisGraphClient, new NullLogger<PolicyRepository>());
var redisPolicyRepository =
    new RedisPolicyRepository(clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var policies = await policyRepository.GetAllAsync();

foreach (var policy in policies)
{
    await redisPolicyRepository.AddAsync(policy);
}

#endregion



#region Task
Console.WriteLine("Writing Tasks...");
var taskRepository = new TaskRepository(multiplexer, redisGraphClient, new NullLogger<TaskRepository>());
var redisTaskRepository =
    new RedisTaskRepository(clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var tasks = await taskRepository.GetAllAsync();

foreach (var task in tasks)
{
    await redisTaskRepository.AddAsync(task);
}

#endregion


#region ActionPlan
Console.WriteLine("Writing ActionPlans...");
var actionPlanRepository = new ActionPlanRepository(multiplexer, redisGraphClient, new NullLogger<ActionPlanRepository>());
var redisActionPlanRepository =
    new RedisActionPlanRepository(clusterConnectionProvider, clusterGraphClient, Serilog.Log.Logger);

var actionPlans = await actionPlanRepository.GetAllAsync();

foreach (var actionPlan in actionPlans)
{
    actionPlan.LastStatusChange = DateTime.Now;
    actionPlan.TaskStartedAt = DateTime.Now.AddDays(-1);
    actionPlan.ActionSequence.ForEach(a=>a.Services.ForEach(i=>i.OnboardedTime = DateTime.Today.AddDays(-10)));
    await redisActionPlanRepository.AddAsync(actionPlan);
}

#endregion


#region relations
Console.WriteLine("Writing Relations...");
var historicalActionPlanRepository = new RedisHistoricalActionPlanRepository(clusterConnectionProvider, redisGraphClient, Serilog.Log.Logger);
var dashboardService = new DashboardService(robotRepository, taskRepository, actionPlanRepository, edgeRepository,
    cloudRepository, instanceRepository, actionRepository, historicalActionPlanRepository);

var relations = await dashboardService.GetAllRelationModelsAsync();

var entities = relations.Entities;
//var types = entities.Select(x => x.Type).Distinct().ToList();

var hashRelations = relations.Relations.ToHashSet(new SimpleRelationComparer());
foreach (var relation in hashRelations)
{
    var initiates = entities.FirstOrDefault(e => e.Id == relation.OriginatingId);
    var points = entities.FirstOrDefault(e => e.Id == relation.PointsToId);
    if (initiates is null || points is null)
    {
        continue;
    }
    var fullRelation = new RelationModel(initiates, points, relation.RelationName);
    await redisActionPlanRepository.AddRelationAsync(fullRelation);
}
#endregion

Console.WriteLine("The data conversion process has finished. Press any key to close.");
Console.ReadLine();
