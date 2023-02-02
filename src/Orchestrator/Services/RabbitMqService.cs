using System.Text;
using Middleware.Common.Config;
using RabbitMQ.Client;
using RabbitMQ.Client.Events;

namespace Middleware.Orchestrator.Services;

public class RabbitMqService : IHostedService
{
    private const string ExchangeName = "deployments";
    private readonly IServiceProvider _serviceProvider;
    private readonly IConnection _connection;
    private readonly IModel _model;

    public RabbitMqService(IServiceProvider serviceProvider RabbitMqConfig config)
    {
        if (config == null)
            throw new ArgumentNullException(nameof(config));

        
        _serviceProvider = serviceProvider;
        var factory = new ConnectionFactory()
        {
            HostName = config.Address,
            UserName = config.User,
            Password = config.Pass
        };
        _connection = factory.CreateConnection();
        _model = _connection.CreateModel();
        _model.ExchangeDeclare(exchange: ExchangeName,
            type: "direct",
            durable: true);
        
        var queueName = _model.QueueDeclare().QueueName;

        _model.QueueBind(queue: queueName,
            exchange: Exchange,
            routingKey: mwName);

        Console.WriteLine($" [*] Waiting for deployments. with routing {mwName}");

        var consumer = new EventingBasicConsumer(_model);
        consumer.Received += (model, ea) =>
        {
            var body = ea.Body.ToArray();
            var message = Encoding.UTF8.GetString(body);
            var routingKey = ea.RoutingKey;
            Console.WriteLine(" [x] Received '{0}':'{1}'",
                routingKey, message);
            _model.BasicAck(ea.DeliveryTag, false);
        };
        channel.BasicConsume(queue: queueName,
            autoAck: false,
            consumer: consumer);

        Console.WriteLine(" Press [enter] to exit.");
        Console.ReadLine();
    }

    public void Dispose()
    {
        _model.Dispose();
        _connection.Dispose();
    }

    public async Task StartAsync(CancellationToken cancellationToken)
    {
        throw new NotImplementedException();
    }

    public async Task StopAsync(CancellationToken cancellationToken)
    {
        throw new NotImplementedException();
    }
}