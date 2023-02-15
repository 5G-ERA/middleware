using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "user-idx", StorageType = StorageType.Json, Prefixes = new[] { UserDto.Prefix })]

public class UserDto : Dto
{
    public const string Prefix = "User";
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;
    [Indexed]
    public string Password { get; init; } = default!;
    [Indexed]
    public string Salt { get; init; } = default!;
    public override BaseModel ToModel()
    {
        var dto = this;
        return new UserModel()
        {
            Id = Guid.Parse(dto.Id.Replace(Prefix, "")),
            Password = dto.Password,
            Salt = dto.Salt
        };
    }

}

