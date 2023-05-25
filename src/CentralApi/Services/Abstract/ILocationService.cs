using System.Collections.Immutable;
using FluentValidation;
using Middleware.Models.Domain;
using OneOf;
using OneOf.Types;

namespace Middleware.CentralApi.Services;

public interface ILocationService
{
    Task<OneOf<Location, ValidationException, NotFound>> RegisterLocation(Location location);

    Task<OneOf<ImmutableList<Location>, NotFound>> GetAvailableLocations(string organization);
}