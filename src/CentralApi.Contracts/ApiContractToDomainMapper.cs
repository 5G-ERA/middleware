using Middleware.CentralApi.Contracts.Responses;
using Middleware.Models.Domain;
using Middleware.Models.Enums;

namespace Middleware.CentralApi.Contracts;

public static class ApiContractToDomainMapper
{
    public static Location ToLocation(this LocationResponse l)
    {
        return new()
        {
            Id = l.Id,
            Name = l.Name,
            Organization = l.Organization,
            Type = Enum.Parse<LocationType>(l.Type),
            IsOnline = l.IsOnline,
            Address = l.Address!
        };
    }
}