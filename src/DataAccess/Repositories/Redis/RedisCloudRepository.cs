using System.Collections.Immutable;
using Microsoft.IdentityModel.Tokens;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories
{
    public class RedisCloudRepository : RedisRepository<CloudModel, CloudDto>, ICloudRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="provider"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public RedisCloudRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(provider, redisGraph, true, logger)
        {

        }

        /// <summary>
        /// Patching properties for CloudModel
        /// </summary>
        /// <param name="id"></param>
        /// <param name="patch"></param>
        /// <returns>Patched model</returns>
        public async Task<CloudModel> PatchCloudAsync(Guid id, CloudModel patch)
        {
            CloudModel? currentModel = await GetByIdAsync(id);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }
            if (!string.IsNullOrEmpty(patch.Type))
            {
                currentModel.Type = patch.Type;
            }
            if (!string.IsNullOrEmpty(patch.CloudStatus))
            {
                currentModel.CloudStatus = patch.CloudStatus;
            }
            if ((patch.CloudIp != null) && Uri.IsWellFormedUriString(patch.CloudIp.ToString(), UriKind.RelativeOrAbsolute))
            {
                currentModel.CloudIp = patch.CloudIp;
            }
            if (!string.IsNullOrEmpty(patch.NumberOfCores.ToString()))
            {
                currentModel.NumberOfCores = patch.NumberOfCores;
            }
            if (!string.IsNullOrEmpty(patch.DiskStorage.ToString()))
            {
                currentModel.DiskStorage = patch.DiskStorage;
            }
            if (!string.IsNullOrEmpty(patch.VirtualRam.ToString()))
            {
                currentModel.VirtualRam = patch.VirtualRam;
            }
            if (!string.IsNullOrEmpty(patch.Cpu.ToString()))
            {
                currentModel.Cpu = patch.Cpu;
            }
            if (!string.IsNullOrEmpty(patch.Ram.ToString()))
            {
                currentModel.Ram = patch.Ram;
            }
            if (!string.IsNullOrEmpty(patch.MacAddress))
            {
                currentModel.MacAddress = patch.MacAddress;
            }
            if (!string.IsNullOrEmpty(patch.LastUpdatedTime.ToString()))
            {
                currentModel.LastUpdatedTime = patch.LastUpdatedTime;
            }
            if (patch.IsOnline != null)
            {
                currentModel.IsOnline = patch.IsOnline;
            }
            await UpdateAsync(currentModel);
            return currentModel;
        }


        public async Task<CloudModel?> GetCloudResourceDetailsByNameAsync(string name)
        {   
            CloudModel? cloud = await FindSingleAsync(dto => dto.Name == name);
            return cloud;
        }

        /// <summary>
        /// Return a list of free clouds that have connectivity to the robot
        /// </summary>
        /// <param name="cloudsToCheck"></param>
        /// <returns></returns>
        public async Task<List<CloudModel>> GetFreeCloudsIdsAsync(List<CloudModel> cloudsToCheck)
        {
            //get all clouds
            List<CloudModel> tempFreeClouds = new List<CloudModel>();

            // Find all clouds that dont have a relationship of type -LOCATED_AT-

            foreach (CloudModel cloud in cloudsToCheck)
            {
                // Get only a list of relatioModel with relations that have a localited_At property and cloud iD.
                List<RelationModel> relations = await GetRelation(
                    cloud.Id,
                    "LOCATED_AT",
                    RelationDirection.Incoming);

                if (relations.IsNullOrEmpty()){
                    tempFreeClouds.Add(cloud);
                }
            }
            return tempFreeClouds;
        }

        /// <summary>
        /// Get less busy cloud list ordered by less busy cloud
        /// </summary>
        /// <param name="busyCloudsToCheck"></param>
        /// <returns>list of cloudModel</returns>
        public async Task<List<CloudModel>> GetLessBusyCloudsAsync(List<CloudModel> busyCloudsToCheck)
        {
            Dictionary<CloudModel, int> tempDic = new Dictionary<CloudModel, int>();
            List<CloudModel> lessBusyClouds = new List<CloudModel>();
            string previousCloud = string.Empty;

            foreach (CloudModel busyCloud in busyCloudsToCheck)
            {
                List<RelationModel> robotRelations = await GetRelation(
                    busyCloud.Id,
                    "LOCATED_AT",
                    RelationDirection.Incoming);

                foreach (RelationModel relationModel in robotRelations)
                {
                    if (relationModel.PointsTo.Name == previousCloud)//Check how many times an edge have the relationship LOCATED_AT
                    {
                        CloudModel cloud = new CloudModel();
                        cloud.Id = relationModel.PointsTo.Id;
                        cloud.Name = relationModel.PointsTo.Name;

                        tempDic[cloud]++;//tempDic.Add(relationModel.PointsTo.Id
                    }
                    previousCloud = relationModel.PointsTo.Name;
                }
            }
            var ordered = tempDic.OrderBy(x => x.Value).ToDictionary(x => x.Key, x => x.Value);  //Order the dictionary by value
            foreach (var element in ordered)
            {
                lessBusyClouds.Add(element.Key);
            }

            return lessBusyClouds;
        }

        /// <summary>
        /// Return number of containers allocated in cloud with specific name
        /// </summary>
        /// <param name="cloudName"></param>
        /// <returns></returns>
        public async Task<int> GetNumContainersByNameAsync(string cloudName)
        {
            CloudModel? cloud = await FindSingleAsync(dto => dto.Name == cloudName);
            if (cloud is null)
                throw new ArgumentException("Cloud does not exist", nameof(cloudName));

            List<RelationModel> cloudRelations = await GetRelation(cloud.Id, "LOCATED_AT", RelationDirection.Incoming);
            return cloudRelations.Count;
        }

        /// <summary>
        /// Return number of containers allocated in cloud with specific id
        /// </summary>
        /// <param name="cloudId"></param>
        /// <returns></returns>
        public async Task<int> GetNumContainersByIdAsync(Guid cloudId)
        {
            List<RelationModel> robotRelations = await GetRelation(cloudId, "LOCATED_AT", RelationDirection.Incoming);
            return robotRelations.Count;
        }

        /// <summary>
        /// Return bool if cloud is busy by cloud Name.
        /// </summary>
        /// <param name="cloudName"></param>
        /// <returns></returns>
        public async Task<bool> IsBusyCloudByNameAsync(string cloudName)
        {
            CloudModel? cloud = await FindSingleAsync(dto => dto.Name == cloudName);
            if (cloud is null)
                throw new ArgumentException("Cloud does not exist", nameof(cloudName));

            List<RelationModel> cloudRelations = await GetRelation(cloud.Id, "LOCATED_AT", RelationDirection.Incoming);

            return cloudRelations.Count > 0;
        }

        /// <summary>
        /// Return bool if cloud is busy by cloud Id.
        /// </summary>
        /// <param name="cloudId"></param>
        /// <returns></returns>
        public async Task<bool> IsBusyCloudByIdAsync(Guid cloudId)
        {
            List<RelationModel> cloudRelations = await GetRelation(cloudId, "LOCATED_AT", RelationDirection.Incoming);
            return cloudRelations.Count > 0;
        }

        /// <summary>
        /// Return all the clouds of a particular organization.
        /// </summary>
        /// <param name="organization"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        public async Task<ImmutableList<CloudModel>> GetCloudsByOrganizationAsync(string organization)
        {
            var matchedClouds = await FindQuery(dto => dto.Organization == organization).ToListAsync();
            return matchedClouds.Select(c=>(CloudModel)c.ToModel()).ToImmutableList();
        }

        /// <summary>
        /// Checks if an edge with a particular name exists.
        /// </summary>
        /// <param name="name"></param>
        /// <returns></returns>
        public async Task<(bool, CloudModel?)> CheckIfNameExists(string name)
        {
            CloudModel? matchedCloud = await FindSingleAsync(dto => dto.Name == name);
            if (matchedCloud is not null)
            {
                return (true, matchedCloud);
            }
            else
            {
                return (false, matchedCloud);
            }
        }

        /// <summary>
        /// Checks if an edge with a particular id exists.
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public async Task<(bool, CloudModel?)> CheckIfIdExists(string id)
        {
            CloudModel? matchedCloud = await FindSingleAsync(dto => dto.Id == id);
            if (matchedCloud is not null)
            {
                return (true, matchedCloud);
            }
            else
            {
                return (false, matchedCloud);
            }
        }
    }
}
