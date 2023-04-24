using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Domain;
using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.Models.Enums;

namespace Middleware.CentralApi.Mappings;

public static class ApiContractToDomainMapper
{
    public static Location ToLocation (this RegisterRequest x)
    {
        return new Location()
        {
            Name = x.Name,
            Organization = x.Organization,
            Type = Enum.Parse<LocationType>(x.Type)
        };
    }
}