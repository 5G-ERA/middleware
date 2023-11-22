using System.Collections.Immutable;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories;

public class RedisEdgeRepository : RedisRepository<EdgeModel, EdgeDto>, IEdgeRepository
{
    /// <summary>
    ///     Default constructor
    /// </summary>
    /// <param name="redisClient"></param>
    /// <param name="redisGraph"></param>
    /// <param name="logger"></param>
    public RedisEdgeRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(
        provider, redisGraph, true, logger)
    {
    }

    /// <summary>
    ///     Patching properties for EdgeModel
    /// </summary>
    /// <param name="id"></param>
    /// <param name="patch"></param>
    /// <returns> Patched model </returns>
    public async Task<EdgeModel> PatchEdgeAsync(Guid id, EdgeModel patch)
    {
        var currentModel = await GetByIdAsync(id);
        if (currentModel == null) return null;
        if (!string.IsNullOrEmpty(patch.Name)) currentModel.Name = patch.Name;
        if (!string.IsNullOrEmpty(patch.EdgeStatus)) currentModel.EdgeStatus = patch.EdgeStatus;
        if (patch.EdgeIp != null && Uri.IsWellFormedUriString(patch.EdgeIp.ToString(), UriKind.RelativeOrAbsolute))
            currentModel.EdgeIp = patch.EdgeIp;
        if (!string.IsNullOrEmpty(patch.MacAddress)) currentModel.MacAddress = patch.MacAddress;
        if (!string.IsNullOrEmpty(patch.Cpu.ToString())) currentModel.Cpu = patch.Cpu;
        if (!string.IsNullOrEmpty(patch.Ram.ToString())) currentModel.Ram = patch.Ram;
        if (!string.IsNullOrEmpty(patch.VirtualRam.ToString())) currentModel.VirtualRam = patch.VirtualRam;
        if (!string.IsNullOrEmpty(patch.DiskStorage.ToString())) currentModel.DiskStorage = patch.DiskStorage;
        if (!string.IsNullOrEmpty(patch.NumberOfCores.ToString())) currentModel.NumberOfCores = patch.NumberOfCores;
        if (!string.IsNullOrEmpty(patch.LastUpdatedTime.ToString()))
            currentModel.LastUpdatedTime = patch.LastUpdatedTime;
        if (patch.IsOnline != null) currentModel.IsOnline = patch.IsOnline;
        await UpdateAsync(currentModel);
        return currentModel;
    }

    public async Task<List<EdgeModel>> GetFreeEdgesIdsAsync(List<EdgeModel> edgesToCheck)
    {
        //get all clouds
        var TempFreeEdges = new List<EdgeModel>();

        // Find all clouds that dont have a relationship of type -LOCATED_AT-

        foreach (var edge in edgesToCheck)
        {
            // Get only a list of relatioModel with relations that have a localited_At property and cloud iD.
            var relations = await GetRelation(
                edge.Id,
                "LOCATED_AT",
                RelationDirection.Incoming);

            if (relations.Count == 0) TempFreeEdges.Add(edge);
        }

        return TempFreeEdges;
    }

    public async Task<List<EdgeModel>> GetLessBusyEdgesAsync(List<EdgeModel> busyEdgesToCheck)
    {
        //Dictionary<Guid, int> tempDic = new Dictionary<Guid,int>();
        var tempDic = new Dictionary<EdgeModel, int>();
        //List<Guid> lessBusyEdges = new List<Guid>();
        var lessBusyEdges = new List<EdgeModel>();
        //int counter = 0;            

        foreach (var busyEdge in busyEdgesToCheck)
        {
            var robotRelations = await GetRelation(busyEdge.Id, "LOCATED_AT", RelationDirection.Incoming);

            var edge = new EdgeModel();
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

        var ordered =
            tempDic.OrderBy(x => x.Value).ToDictionary(x => x.Key, x => x.Value); //Order the dictionary by value
        foreach (var element in ordered)
        {
            lessBusyEdges.Add(element.Key);
        }

        return lessBusyEdges;
    }

    public async Task<EdgeModel?> GetEdgeResourceDetailsByNameAsync(string name)
    {
        var edge = await FindSingleAsync(dto => dto.Name == name);
        return edge;
    }

    /// <summary>
    ///     Return bool if edge is busy by edge Id.
    /// </summary>
    /// <param name="cloudName"></param>
    /// <returns></returns>
    public async Task<bool> IsBusyEdgeByIdAsync(Guid edgeId)
    {
        var edgeRelations = await GetRelation(edgeId, "LOCATED_AT", RelationDirection.Incoming);
        return edgeRelations.Count > 0;
    }

    /// <summary>
    ///     Return bool if edge is busy by edge name.
    /// </summary>
    /// <param name="cloudName"></param>
    /// <returns></returns>
    public async Task<bool> IsBusyEdgeByNameAsync(string edgeName)
    {
        var edge = await FindSingleAsync(dto => dto.Name == edgeName);
        if (edge is null)
            throw new ArgumentException("Edge does not exist", nameof(edgeName));
        var edgeRelations = await GetRelation(edge.Id, "LOCATED_AT", RelationDirection.Incoming);
        return edgeRelations.Count > 0;
    }

    /// <summary>
    ///     Return number of containers alocated in edge with specific id
    /// </summary>
    /// <param name="edgeId"></param>
    /// <returns></returns>
    public async Task<int> GetNumContainersByIdAsync(Guid edgeId)
    {
        var edgeRelations = await GetRelation(edgeId, "LOCATED_AT", RelationDirection.Incoming);
        return edgeRelations.Count;
    }

    /// <summary>
    ///     Return number of containers alocated in edge with specific name
    /// </summary>
    /// <param name="edgeName"></param>
    /// <returns></returns>
    public async Task<int> GetNumContainersByNameAsync(string edgeName)
    {
        var edge = await FindSingleAsync(dto => dto.Name == edgeName);
        if (edge is null)
            throw new ArgumentException("Edge does not exist", nameof(edgeName));
        var robotRelations = await GetRelation(edge.Id, "LOCATED_AT", RelationDirection.Incoming);
        return robotRelations.Count;
    }

    /// <summary>
    ///     Return all the edges of a particular organization.
    /// </summary>
    /// <param name="organization"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentException"></exception>
    public async Task<ImmutableList<EdgeModel>> GetEdgesByOrganizationAsync(string organization)
    {
        var matchedEdges = await FindAsync(dto => dto.Organization == organization);
        return matchedEdges.ToImmutableList();
    }


    /// <summary>
    ///     Check if a given address is stored in redis for the edges entities.
    /// </summary>
    /// <param name="address"></param>
    /// <returns></returns>
    public async Task<bool> CheckIfAddressExists(Uri address)
    {
        var matchedEdge = await FindSingleAsync(dto => dto.EdgeIp == address);
        if (matchedEdge is not null)
            return true;
        return false;
    }

    /// <summary>
    ///     Checks if an edge exists with a particular name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    public async Task<(bool, EdgeModel?)> CheckIfNameExists(string name)
    {
        var matchedEdge = await FindSingleAsync(dto => dto.Name == name);
        if (matchedEdge is not null)
            return (true, matchedEdge);
        return (false, matchedEdge);
    }

    /// <summary>
    ///     Checks if an edge exists with a particular id.
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    public async Task<(bool, EdgeModel?)> CheckIfIdExists(string id)
    {
        var matchedEdge = await FindSingleAsync(dto => dto.Id == id);
        if (matchedEdge is not null)
            return (true, matchedEdge);
        return (false, matchedEdge);
    }

    /// <summary>
    ///     Return online status and LastUpdatedTime of edge by cloud Id.
    /// </summary>
    /// <param name="edgeId"></param>
    /// <returns>Return bool of isOnline and dateTime of lastUpdatedTime of the edgeModel</returns>
    public async Task<CloudEdgeStatusResponse> GetEdgeOnlineStatusLastUpdatedTimeAsync(Guid edgeId)
    {
        var edge = await GetByIdAsync(edgeId);
        if (edge is null)
            throw new ArgumentException("Edge does not exist", nameof(edgeId));
        var response = new CloudEdgeStatusResponse();
        response.Id = edgeId;
        response.LastUpdatedTime = edge.LastUpdatedTime;
        response.IsOnline = edge.IsOnline;
        response.Type = "Edge";

        return response;
    }

    /// <summary>
    ///     Change online status of edge by edge Id.
    /// </summary>
    /// <param name="edgeId"></param>
    /// <param name="isOnline"></param>
    /// <returns></returns>
    public async Task SetEdgeOnlineStatusAsync(Guid edgeId, bool isOnline)
    {
        var edge = await GetByIdAsync(edgeId);
        if (edge is null)
            throw new ArgumentException("Edge does not exist", nameof(edgeId));
        edge.IsOnline = isOnline;
        if (edge.IsOnline) edge.LastUpdatedTime = DateTime.UtcNow;
        _ = await AddAsync(edge) ?? throw new ArgumentException("Edge does not exist", nameof(edge));
    }
}