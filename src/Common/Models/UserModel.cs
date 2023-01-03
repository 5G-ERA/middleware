using System.ComponentModel.DataAnnotations;
using System.Text.Json.Serialization;

namespace Middleware.Common.Models
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

    }
}
