﻿using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Ros;

[Document]
public class Actuator
{
    [Indexed]
    public string? Name { get; set; }
    [Indexed]
    public string? Type { get; set; }
    [Indexed(Sortable = true)]
    public int Number { get; set; }
    [Indexed]
    public List<string> Nodes { get; set; } = new();
}