using FluentValidation;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.CentralApi.Domain;
using Middleware.CentralApi.Services;
using OneOf;
using OneOf.Types;

namespace Middleware.CentralApi.Services;

public class LocationService : ILocationService
{
    public OneOf<RegistrationResult, ValidationException, NotFound> RegisterLocation()
    {
        // when location not found in db
        return new NotFound();
        
        // when location is not valid 
        return new ValidationException("The specified location is not valid");
        
        
        // when ok
        return new RegistrationResult(true);

    }

    public async Task<OneOf<List<Location>, NotFound>> GetAvailableLocations()
    {
        throw new NotImplementedException();
    }
}