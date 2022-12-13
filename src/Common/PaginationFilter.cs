using System;
using System.Linq;

namespace Middleware.Common;

public class PaginationFilter
{
    public int PageNumber { get; set; }
    public int PageSize { get; set; }
    public PaginationFilter()
    {
        PageNumber = 1;
        PageSize = 10;
    }
    public PaginationFilter(int pageNumber, int pageSize)
    {
        PageNumber = pageNumber < 1 ? 1 : pageNumber;
        PageSize = pageSize > 10 ? 10 : pageSize;
    }

    /// <summary>
    /// Filters the result list based on the current filter configuration
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="list">List to be filtered</param>
    /// <returns>The list with specified in filter number of elements and skips first n pages</returns>
    public List<T> FilterResult<T>(List<T> list)
    {
        return list.Skip((PageNumber - 1) * PageSize)
                .Take(PageSize)
                .ToList();
    }
}
