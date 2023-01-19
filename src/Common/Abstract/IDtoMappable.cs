namespace Middleware.Common.Abstract
{
    public interface IDtoMappable<TDto>
    {
        TDto MapToDto();
    }
}
