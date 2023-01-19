using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface ICloudRepository : IBaseRepository<CloudModel>, IRelationRepository
    {
        Task<CloudModel> PatchCloudAsync(Guid id, CloudModel patch);

        Task<CloudModel> GetCloudResourceDetailsByNameAsync(string name);

        Task<List<CloudModel>> GetFreeCloudsIdsAsync(List<CloudModel> cloudsToCheck);

        Task<List<CloudModel>> GetLessBusyCloudsAsync(List<CloudModel> busyCloudsTocheck);

        Task<int> GetNumContainersByNameAsync(string cloudName);

        Task<int> GetNumContainersByIdAsync(Guid cloudId);

        Task<bool> IsBusyCloudByNameAsync(string cloudName);

        Task<bool> IsBusyCloudByIdAsync(Guid cloudId);
    }
}
