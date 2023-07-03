namespace Middleware.Common;

public interface IUriService
{
    Uri GetPageUri(PaginationFilter filter, string route);
}