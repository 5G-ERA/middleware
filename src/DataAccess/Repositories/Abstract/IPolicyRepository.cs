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
        Task<PolicyModel?> PatchPolicyAsync(Guid id, PolicyModel patch);
        /// <summary>
        /// Gets the active system-scoped policies
        /// </summary>
        /// <returns></returns>
        Task<IReadOnlyCollection<PolicyModel>> GetActiveSystemPoliciesAsync();
    }
}
