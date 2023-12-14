using System.ComponentModel.DataAnnotations;
using System.Text.Json.Serialization;
using Middleware.Models.Dto;

namespace Middleware.Models.Domain;

public class UserModel : BaseModel
{
    [Required]
    public override Guid Id { get; set; } = Guid.NewGuid();

    [Required]
    public string Password { get; set; } = default!;

    [JsonPropertyName("UserName")]
    public override string Name { get; set; } = default!;

    public string Salt { get; set; } = default!;

    public string Role { get; set; } = "robot";

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new UserDto
        {
            Id = domain.Id.ToString(),
            Password = domain.Password,
            Salt = domain.Salt,
            Role = domain.Role,
            UserName = domain.Name
        };
    }
}