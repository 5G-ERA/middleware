﻿using System.Text.Json;
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
using ILogger = Serilog.ILogger;

namespace Middleware.DataAccess.Repositories.Redis
{
    public class RedisEdgeRepository : RedisRepository<EdgeModel, EdgeDto>, IEdgeRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public RedisEdgeRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(provider, redisGraph, true, logger)
        {

        }

        /// <summary>
        /// Patching properties for EdgeModel
        /// </summary>
        /// <param name="id"></param>
        /// <param name="patch"></param>
        /// <returns> Patched model </returns>
        public async Task<EdgeModel> PatchEdgeAsync(Guid id, EdgeModel patch)
        {
            EdgeModel? currentModel = await GetByIdAsync(id);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }
            if (!string.IsNullOrEmpty(patch.EdgeStatus))
            {
                currentModel.EdgeStatus = patch.EdgeStatus;
            }
            if ((patch.EdgeIp != null) && Uri.IsWellFormedUriString(patch.EdgeIp.ToString(), UriKind.RelativeOrAbsolute))
            {
                currentModel.EdgeIp = patch.EdgeIp;
            }
            if (!string.IsNullOrEmpty(patch.MacAddress))
            {
                currentModel.MacAddress = patch.MacAddress;
            }
            if (!string.IsNullOrEmpty(patch.Cpu.ToString()))
            {
                currentModel.Cpu = patch.Cpu;
            }
            if (!string.IsNullOrEmpty(patch.Ram.ToString()))
            {
                currentModel.Ram = patch.Ram;
            }
            if (!string.IsNullOrEmpty(patch.VirtualRam.ToString()))
            {
                currentModel.VirtualRam = patch.VirtualRam;
            }
            if (!string.IsNullOrEmpty(patch.DiskStorage.ToString()))
            {
                currentModel.DiskStorage = patch.DiskStorage;
            }
            if (!string.IsNullOrEmpty(patch.NumberOfCores.ToString()))
            {
                currentModel.NumberOfCores = patch.NumberOfCores;
            }
            await UpdateAsync(currentModel);
            return currentModel;
        }

        public async Task<List<EdgeModel>> GetFreeEdgesIdsAsync(List<EdgeModel> edgesToCheck)
        {
            //List<Guid> TempFreeEdges = new List<Guid>();
            List<EdgeModel> TempFreeEdges = new List<EdgeModel>();
            //edgesToCheck.AddRange(TempFreeEdges);
            //TempFreeEdges = edgesToCheck;

            foreach (EdgeModel edgeId in edgesToCheck)
            {
                //Get edge id from name
                List<RelationModel> robotRelations = await GetRelation(edgeId.Id, "LOCATED_AT", RelationDirection.Incoming);

                foreach (RelationModel relationModel in robotRelations)
                {
                    if (relationModel.PointsTo != null)
                    {
                        EdgeModel edge = new EdgeModel();
                        edge.Id = relationModel.PointsTo.Id;
                        edge.Name = relationModel.PointsTo.Name;
                        //TempFreeEdges.Remove(relationModel.PointsTo.Id);
                        TempFreeEdges.Add(edge);
                    }
                }
            }
            return TempFreeEdges;
        }

        public async Task<List<EdgeModel>> GetLessBusyEdgesAsync(List<EdgeModel> busyEdgesToCheck)
        {
            //Dictionary<Guid, int> tempDic = new Dictionary<Guid,int>();
            Dictionary<EdgeModel, int> tempDic = new Dictionary<EdgeModel, int>();
            //List<Guid> lessBusyEdges = new List<Guid>();
            List<EdgeModel> lessBusyEdges = new List<EdgeModel>();
            //int counter = 0;            

            foreach (EdgeModel busyEdge in busyEdgesToCheck)
            {
                List<RelationModel> robotRelations = await GetRelation(busyEdge.Id, "LOCATED_AT", RelationDirection.Incoming);

                EdgeModel edge = new EdgeModel();
                edge.Id = busyEdge.Id;
                edge.Name = busyEdge.Name;
                tempDic.Add(edge, robotRelations.Count);
                /*
                foreach (RelationModel relationModel in robotRelations)
                {   
                    if (relationModel.PointsTo.Name == previousEdge)//Check how many times an edge have the relationship LOCATED_AT
                    {
                        EdgeModel edge = new EdgeModel();
                        edge.Id = relationModel.PointsTo.Id;
                        edge.Name = relationModel.PointsTo.Name;

                        tempDic[edge]++;//tempDic.Add(relationModel.PointsTo.Id
                    }
                    previousEdge = relationModel.PointsTo.Name;

                }*/
            }
            var ordered = tempDic.OrderBy(x => x.Value).ToDictionary(x => x.Key, x => x.Value);  //Order the dictionary by value
            foreach (var element in ordered)
            {
                lessBusyEdges.Add(element.Key);
            }

            return lessBusyEdges;
        }

        public async Task<EdgeModel> GetEdgeResourceDetailsByNameAsync(string name)
        {
            EdgeModel edge = await FindSingleAsync(dto => dto.Name == name);
            return edge;
        }

        /// <summary>
        /// Return bool if edge is busy by edge Id.
        /// </summary>
        /// <param name="cloudName"></param>
        /// <returns></returns>
        public async Task<bool> IsBusyEdgeByIdAsync(Guid edgeId)
        {
            List<RelationModel> edgeRelations = await GetRelation(edgeId, "LOCATED_AT", RelationDirection.Incoming);
            return edgeRelations.Count > 0;
        }

        /// <summary>
        /// Return bool if edge is busy by edge name.
        /// </summary>
        /// <param name="cloudName"></param>
        /// <returns></returns>
        public async Task<bool> IsBusyEdgeByNameAsync(string edgeName)
        {
            EdgeModel edge = await FindSingleAsync(dto => dto.Name == edgeName);
            if (edge is null)
                throw new ArgumentException("Edge does not exist", nameof(edgeName));
            List<RelationModel> edgeRelations = await GetRelation(edge.Id, "LOCATED_AT", RelationDirection.Incoming);
            return edgeRelations.Count > 0;
        }

        /// <summary>
        /// Return number of containers alocated in edge with specific id
        /// </summary>
        /// <param name="edgeId"></param>
        /// <returns></returns>
        public async Task<int> GetNumContainersByIdAsync(Guid edgeId)
        {
            List<RelationModel> edgeRelations = await GetRelation(edgeId, "LOCATED_AT", RelationDirection.Incoming);
            return edgeRelations.Count;
        }

        /// <summary>
        /// Return number of containers alocated in edge with specific name
        /// </summary>
        /// <param name="edgeName"></param>
        /// <returns></returns>
        public async Task<int> GetNumContainersByNameAsync(string edgeName)
        {
            EdgeModel edge = await FindSingleAsync(dto => dto.Name == edgeName);
            if (edge is null)
                throw new ArgumentException("Edge does not exist", nameof(edgeName));
            List<RelationModel> robotRelations = await GetRelation(edge.Id, "LOCATED_AT", RelationDirection.Incoming);
            return robotRelations.Count;
        }
    }
}
