using System.Text.Json;
using System.Xml.Linq;
using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Enums;
using NReJSON;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.DataAccess.Repositories.Redis
{
    public class RedisCloudRepository : RedisRepository<CloudModel, CloudDto>, ICloudRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public RedisCloudRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph) : base(provider, redisGraph, true)
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
            if (!string.IsNullOrEmpty(patch.CloudStatus))
            {
                currentModel.CloudStatus = patch.CloudStatus;
            }
            if ((patch.CloudIp != null) && Uri.IsWellFormedUriString(patch.CloudIp.ToString(), UriKind.RelativeOrAbsolute))
            {
                currentModel.CloudIp = patch.CloudIp;
            }
            await UpdateAsync(currentModel);
            return currentModel;
        }


        public async Task<CloudModel> GetCloudResourceDetailsByNameAsync(string name)
        {   
            CloudModel cloud = await FindSingleAsync(dto => dto.Name == name);
            return cloud;
        }

        /// <summary>
        /// Return a list of free clouds that have connectivity to the robot
        /// </summary>
        /// <param name="cloudsToCheck"></param>
        /// <returns></returns>
        public async Task<List<CloudModel>> GetFreeCloudsIdsAsync(List<CloudModel> cloudsToCheck)
        {
            List<CloudModel> TempFreeClouds = new List<CloudModel>();

            foreach (CloudModel cloudId in cloudsToCheck)
            {
                // Get cloud id from name
                List<RelationModel> robotRelations = await GetRelation(
                    cloudId.Id,
                    "LOCATED_AT",
                    RelationDirection.Incoming);

                foreach (RelationModel relationModel in robotRelations)
                {
                    if (relationModel.PointsTo != null)
                    {
                        CloudModel cloud = new CloudModel();
                        cloud.Id = relationModel.PointsTo.Id;
                        cloud.Name = relationModel.PointsTo.Name;
                        TempFreeClouds.Add(cloud);
                    }
                }
            }
            return TempFreeClouds;
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
            CloudModel cloud = await FindSingleAsync(dto => dto.Name == cloudName);
            if (cloud is null)
                throw new ArgumentException("Cloud does not exist", nameof(cloudName));

            List<RelationModel> cloudRelations = await GetRelation(cloud.Id, "LOCATED_AT", RelationDirection.Incoming);
            return cloudRelations.Count;
        }

        /// <summary>
        /// Return number of containers allocated in cloud with specific id
        /// </summary>
        /// <param name="cloudName"></param>
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
            CloudModel cloud = await FindSingleAsync(dto => dto.Name == cloudName);
            if (cloud is null)
                throw new ArgumentException("Cloud does not exist", nameof(cloudName));

            List<RelationModel> cloudRelations = await GetRelation(cloud.Id, "LOCATED_AT", RelationDirection.Incoming);

            return cloudRelations.Count > 0;
        }

        /// <summary>
        /// Return bool if cloud is busy by cloud Id.
        /// </summary>
        /// <param name="cloudName"></param>
        /// <returns></returns>
        public async Task<bool> IsBusyCloudByIdAsync(Guid cloudId)
        {
            List<RelationModel> cloudRelations = await GetRelation(cloudId, "LOCATED_AT", RelationDirection.Incoming);
            return cloudRelations.Count > 0;
        }
    }
}
