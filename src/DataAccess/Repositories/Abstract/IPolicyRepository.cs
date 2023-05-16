using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IPolicyRepository : IBaseRepository<PolicyModel>, IRelationRepository
    {
        /// <summary>
        /// Get all active policies
        /// </summary>
        /// <returns></returns>
        Task<List<PolicyModel>> GetActivePoliciesAsync();
        /// <summary>
        /// Returns the policy by its name
        /// </summary>
        /// <param name="name"></param>
        /// <returns>May return null if policy is not found</returns>
        Task<PolicyModel?> GetPolicyByName(string name);
        Task<PolicyModel?> PatchPolicyAsync(Guid id, PolicyModel patch);
        /// <summary>
        /// Gets the active system-scoped policies
        /// </summary>
        /// <returns></returns>
        Task<IReadOnlyCollection<PolicyModel>> GetActiveSystemPoliciesAsync();
    }
}
