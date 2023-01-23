namespace Middleware.Models.Dto;

public abstract class Dto
{
    public abstract string Id { get; set; }

    public abstract object ToModel();
}