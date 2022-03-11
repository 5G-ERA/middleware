using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories
{
    public interface IPolicyRepository : IBaseRepository<PolicyModel>
    {
        /// <summary>
        /// Get all policies
        /// </summary>
        /// <returns></returns>
        Task<List<PolicyModel>> GetAllPoliciesAsync();
        /// <summary>
        /// Get all active policies
        /// </summary>
        /// <returns></returns>
        Task<List<PolicyModel>> GetActivePoliciesAsync();
    }
}
