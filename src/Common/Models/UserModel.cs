using System.ComponentModel.DataAnnotations;

namespace Middleware.Common.Models
{
    public class UserModel : BaseModel
    {
        [Required]
        public override Guid Id { get; set; }

        [Required]
        public string Password { get; set; }
        
        public string Salt { get; set; }    
        
    }
}
