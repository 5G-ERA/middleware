﻿namespace Middleware.Common.Config;

public class RabbitMqConfig
{
    /// <summary>
    /// Name of the section to be parsed
    /// </summary>
    public const string ConfigName = "RabbitMQ";
    /// <summary>
    /// RabbitMQ broker address
    /// </summary>
    public string Address { get; set; }

    /// <summary>
    /// Password
    /// </summary>
    public ushort? Port { get; set; }
    /// <summary>
    /// User
    /// </summary>
    public string User { get; set; }
    /// <summary>
    /// Password
    /// </summary>
    public string Pass { get; set; }
}