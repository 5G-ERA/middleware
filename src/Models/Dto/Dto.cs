namespace Middleware.Models.Dto;

public abstract class Dto
{
    public abstract string Id { get; init; }

    public abstract object ToModel();
}