using FluentValidation;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.CentralApi.Domain;
using OneOf;
using OneOf.Types;

namespace Middleware.CentralApi.Services;

public interface ILocationService
{
    OneOf<RegistrationResult, ValidationException, NotFound> RegisterLocation();
    
    Task<OneOf<List<Location>, NotFound>> GetAvailableLocations();
}