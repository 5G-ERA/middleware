﻿namespace Middleware.RedisInterface.Contracts.Responses;

public class RosTopicResponse
{
    public string Name { get; set; }
    public string Type { get; set; }
    public string Description { get; set; }
    public bool Enabled { get; set; }
}