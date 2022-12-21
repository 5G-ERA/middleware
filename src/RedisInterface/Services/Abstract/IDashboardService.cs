using Middleware.Common;
using Middleware.RedisInterface.Responses;

namespace Middleware.RedisInterface.Services
{
    public interface IDashboardService
    {
        /// <summary>
        /// Gets the list of tasks that were executed by the robots
        /// </summary>
        /// <param name="filter"></param>
        /// <returns>A tuple of values: <br/>
        ///     First: A list of <seealso cref="TaskRobotResponse"/> <br/>
        ///     Second: Total number of records</returns>
        Task<Tuple<List<TaskRobotResponse>, int>> GetRobotStatusListAsync(PaginationFilter filter);

        /// <summary>
        /// Gets the list the locations registered with the Middleware (Clouds and Edges)
        /// </summary>
        /// <param name="filter"></param>
        /// <returns>A tuple of values: <br/>
        ///     First: A list of <seealso cref="LocationStatusResponse"/> <br/>
        ///     Second: Total number of records</returns>
        Task<Tuple<List<LocationStatusResponse>, int>> GetLocationsStatusListAsync(PaginationFilter filter);

        Task<List<actionSequenceResponse>> GetActionSequenceAsync();
    }
}
