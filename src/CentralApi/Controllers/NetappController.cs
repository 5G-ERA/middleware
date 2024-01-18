using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract.Influx;
using Middleware.DataAccess.Repositories.Influx;
using Middleware.Models.Domain;
using ILogger = Serilog.ILogger;
//using Microsoft.Extensions.Logging;


namespace Middleware.CentralApi.Controllers;
[ApiController]
[Route("api/v1/[controller]")]
public class NetappController : Controller
{
    //protected IInfluxRepository<NetAppStatusModel, NetAppStatusDto> Client { get; }
    private readonly ILogger _logger;
    protected IInfluxNetAppStatusRepository NetAppClient;
    protected IInfluxRobotStatusRepository RobotClient;

    public NetappController(ILogger logger,
        IInfluxNetAppStatusRepository netappClient,
        IInfluxRobotStatusRepository robotClient
       )
    {
        NetAppClient = netappClient;
        RobotClient = robotClient;
        _logger = logger;
    }

    /// <summary>
    ///     Creates a new relation between two models
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("org", Name = "AddNewOrganisationAsync")]
    public async Task AddNewOrganisationAsync(string organisationName)
    {
        await NetAppClient.AddOrgAsync(organisationName);
    }
    /*
    [HttpGet]
    [Route("bucket", Name = "AddNewBucketAsync")]
    public async Task AddNewBucketAsync(string bucketName)
    {
        await NetAppClient.AddBucketAsync(bucketName, 3600);
    }
    //*/
    [HttpGet]
    [Route("sample", Name = "AddNewBucketAsync")]
    public async Task GetLastSampleAsync()
    {
        //await NetAppClient.AddBucketAsync(bucketName, 3600);
        await NetAppClient.GetLastByIdAsyncTest();
    }

    /// <summary>
    ///     Add a new RobotStatusModel sample
    /// </summary>
    /// <param name="model"></param>
    /// <returns> the newly created RobotStatusModel entity </returns>
    [HttpPost]
    [Route("netapp", Name = "AddNetAppStatusAsync")]
    [ProducesResponseType(typeof(NetAppStatusModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<NetAppStatusModel>> AddNetAppStatusAsync([FromBody] NetAppStatusModel model)
    {
        if (model.IsValid() == false)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        try
        {
            await NetAppClient.AddOneAsync(model);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            //_logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
        return Ok(model);
    }
    /// <summary>
    ///     Add a new RobotStatusModel sample
    /// </summary>
    /// <param name="model"></param>
    /// <returns> the newly created RobotStatusModel entity </returns>
    [HttpPost]
    [Route("robot", Name = "AddRobotStatusAsync")]
    [ProducesResponseType(typeof(RobotStatusModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<RobotStatusModel>> AddRobotStatusAsync([FromBody] RobotStatusModel model)
    {
        if (model.IsValid() == false)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        try
        {
            await RobotClient.AddOneAsync(model);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            //_logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
        return Ok(model);
    }
}
