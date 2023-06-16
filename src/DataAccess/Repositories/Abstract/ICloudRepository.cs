using System.Collections.Immutable;
using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface ICloudRepository : IRedisRepository<CloudModel, CloudDto>, IRelationRepository
    {
        Task<CloudModel> PatchCloudAsync(Guid id, CloudModel patch);

        Task<CloudModel?> GetCloudResourceDetailsByNameAsync(string name);

        Task<List<CloudModel>> GetFreeCloudsIdsAsync(List<CloudModel> cloudsToCheck);

        Task<List<CloudModel>> GetLessBusyCloudsAsync(List<CloudModel> busyCloudsTocheck);

        Task<int> GetNumContainersByNameAsync(string cloudName);

        Task<int> GetNumContainersByIdAsync(Guid cloudId);

        Task<bool> IsBusyCloudByNameAsync(string cloudName);

        Task<bool> IsBusyCloudByIdAsync(Guid cloudId);
        Task<ImmutableList<CloudModel>> GetCloudsByOrganizationAsync(string organization);

        Task<(bool, CloudModel?)> CheckIfNameExists(string name);

        Task<(bool, CloudModel?)> CheckIfIdExists(string id);

    }
}
