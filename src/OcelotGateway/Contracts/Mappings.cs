using Middleware.Models.Domain;

namespace Middleware.OcelotGateway.Contracts;

public static class Mappings
{
    public static UserModel ToUser(this RegisterRequest x)
    {
        return new UserModel
        {
            Name = x.UserName,
            Password = x.Password,
            Role = x.Role,
        };
    }

    public static RegisterResponse ToRegisterResponse(this UserModel x)
    {
        return new RegisterResponse
        {
            Id = x.Id,
            UserName = x.Name,
            Role = x.Role
        };
    }
}
