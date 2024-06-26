﻿using System.Collections.Immutable;
using Middleware.Models.Domain;
using OneOf;
using OneOf.Types;

namespace Middleware.CentralApi.Services;

public interface ILocationService
{
    Task<OneOf<Location, NotFound>> RegisterLocation(Location location);

    Task<OneOf<ImmutableList<Location>, NotFound>> GetAvailableLocations(string organization);
}