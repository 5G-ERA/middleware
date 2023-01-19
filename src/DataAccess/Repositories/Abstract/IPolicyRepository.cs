using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IPolicyRepository : IBaseRepository<PolicyModel>, IRelationRepository
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

        Task<PolicyModel> PatchPolicyAsync(Guid id, PolicyModel patch);
    }
}
