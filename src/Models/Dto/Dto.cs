using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

public abstract class Dto
{
    [Indexed] 
    public abstract string Id { get; set; }

    public abstract BaseModel ToModel();
    
}