using Middleware.Models.Dto.Hardware;
using Middleware.Models.Dto;
using System.ComponentModel.DataAnnotations;
using System.Text.Json.Serialization;

namespace Middleware.Models.Domain
{
    public class UserModel : BaseModel
    {
        [Required]
        public override Guid Id { get; set; }

        [Required]
        public string Password { get; set; }

        [JsonPropertyName("UserName")]
        public override string Name { get; set; }
        public string Salt { get; set; }

        public string Role { get; set; }
        public override Dto.Dto ToDto()
        {
            var domain = this;
            return new UserDto()
            {
                Id = domain.Id.ToString(),
                Password = domain.Password,
                Salt = domain.Salt,
                Role = domain.Role
                
            };
        }
    }
}