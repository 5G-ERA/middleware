using System.Collections.Generic;
using InfluxDB.Client;
using InfluxDB.Client.Api.Domain;
using InfluxDB.Client.Core.Flux.Domain;
using InfluxDB.Client.Writes;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Exceptions;
using ILogger = Serilog.ILogger;


namespace Middleware.DataAccess.Repositories.Influx;
public class InfluxRepository<TModel, TDto> : IInfluxRepository<TModel, TDto> where TModel : BaseModel where TDto : InfluxDto
{
    private const string Organization = "testorg";
    protected readonly string Bucket;
    protected readonly string ObjectType;
    /// <summary>
    /// Logger instance
    /// </summary>
    protected readonly ILogger Logger;

    /// <summary>
    /// Influx Client
    /// </summary>
    protected readonly IInfluxDBClient Client;

    /// <summary>
    /// Default c-tor
    /// </summary>
    /// <param name="client">Influx client </param>
    /// <param name="logger">Logger instance</param>
    /// <exception cref="ArgumentNullException"></exception>
    public InfluxRepository(IInfluxDBClient client, ILogger logger, string bucket, string objectType)
    {
        Logger = logger ?? throw new ArgumentNullException(nameof(logger));
        Client = client ?? throw new ArgumentNullException(nameof(client));
        Bucket = bucket;
        ObjectType = objectType;
    }

    public async Task AddOrgAsync(string organisationName)
    {
        await Client.GetOrganizationsApi().CreateOrganizationAsync(name: organisationName);
    }

    public async Task<Bucket> AddBucketAsync(string bucketName, int retentionTime)
    {
        var org = Organization;
        var orgId = (await Client.GetOrganizationsApi().FindOrganizationsAsync(org: org)).First().Id;
        var retention = new BucketRetentionRules(BucketRetentionRules.TypeEnum.Expire, retentionTime);
        var bucket = await Client.GetBucketsApi().CreateBucketAsync(bucketName, retention, orgId);
        return bucket;
    }

    public async Task AddPointAsync(PointData point, string bucket, string org)
    {
        //*
        point = PointData.Measurement("heartbeat")
            .Tag("robot", "robot55")
            .Tag("id", "3fa85f64-5717-4562-b3fc-2c963f66afa6")
            .Field("cpu", 55)
            .Field("ram", 2584)
            .Timestamp(DateTime.UtcNow, WritePrecision.Ns);
        //*/
        await Client.GetWriteApiAsync().WritePointAsync(point: point, bucket: bucket, org: org);
    }

    public Task<TModel?> AddAsync(TModel model)
    {
        throw new NotImplementedException();
    }

    /// <summary>
    ///     Add to influx a sample from model.
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    public async Task<TModel?> AddOneAsync(TModel model)
    {
        var dto = ToTDto(model);
        var point = dto.ToPointData();
        await Client.GetWriteApiAsync().WritePointAsync(point: point, bucket: Bucket, org: Organization);
        return ToTModel(dto);
    }
    public async Task<TModel?> GetStatusByIdAsync(Guid id)
    {
        //var dto = await Collection.FindByIdAsync(id.ToString());
        //if (dto is null) return null;
        //return ToTModel(dto);

        var stringId = id.ToString();
        string query = "from(bucket: \"" + ObjectType + "\") |> range(start: 0) |> filter(fn: (r) => r[\"_measurement\"] == \"heartbeat\") |> filter(fn: (r) => r[\"id\"] == \"" + stringId + "\") |> yield(name: \"last\")";
        List<FluxTable> fluxTables = await Client.GetQueryApi().QueryAsync(query: query, org: Organization);
        return FromInfluxDataToModel(fluxTables, fluxTables);
    }
    public async Task GetLastByIdAsyncTest()
    {
        //var dto = await Collection.FindByIdAsync(id.ToString());
        //if (dto is null) return null;
        //return ToTModel(dto);

        /*
        string queryw = "from(bucket: \"RobotStatus\") |> range(start: 0) |> filter(fn: (r) => r[\"_measurement\"] == \"heartbeat\") |> filter(fn: (r) => r[\"id\"] == \"3fa85f64-5717-4562-b3fc-2c963f66afa1\") |> yield(name: \"last\")";
        //*/
        /*
        from(bucket: "RobotStatus")
  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
  |> filter(fn: (r) => r["_measurement"] == "heartbeat")
  |> filter(fn: (r) => r["id"] == "3fa85f64-5717-4562-b3fc-2c963f66afa1")
  |> aggregateWindow(every: v.windowPeriod, fn: last, createEmpty: false)
  |> yield(name: "last")

  //*/

        string flux2 = "from(bucket: \"NetAppStatus\") |> range(start: 0) |> filter(fn: (r) => r[\"_measurement\"] == \"heartbeat\") |> filter(fn: (r) => r[\"id\"] == \"3fa85f64-5717-4562-b3fc-2c963f66afa9\") |> yield(name: \"last\")";
        string queryw = "from(bucket: \"RobotStatus\") |> range(start: 0) |> filter(fn: (r) => r[\"_measurement\"] == \"heartbeat\") |> filter(fn: (r) => r[\"id\"] == \"3fa85f64-5717-4562-b3fc-2c963f66afa1\") |> yield(name: \"last\")";

        //string flux = "from(bucket: 'NetAppStatus') |> range(start: 0) |> filter(fn: (r) => r['_measurement'] == 'heartbeat') |> filter(fn: (r) => r['id'] == '3fa85f64-5717-4562-b3fc-2c963f66afa6') |> yield(name: 'last')";
        //var fluxTables = await Client.GetQueryApi().QueryAsync(query: flux2, org: Organization);

        List<FluxTable> fluxTables = await Client.GetQueryApi().QueryAsync(query: queryw, org: Organization);
        var timee1 = fluxTables[0].Records[0].GetTime().ToString();
        var flx251 = fluxTables[0].Records[0].GetTime;
        var flx261 = fluxTables[0].Records[0].GetTimeInDateTime;
        var flx221 = fluxTables[0].Records[0].GetValueByKey("robot");
        var flx2221 = fluxTables[0].Records[0].GetValueByKey("id");




        foreach (var fluxTable in fluxTables)
        {
            foreach(var fluxrecord in fluxTable.Records)
            {
                var flx23 = fluxrecord.GetMeasurement();
                var flx24 = fluxrecord.GetType();

                var fieldNamee = fluxrecord.GetField().ToString();
                var valuee = fluxrecord.GetValue().ToString();

            }
            //new System.Collections.Generic.ICollectionDebugView<object>(fluxrecord.Row).Items[9]
        }

        fluxTables.ForEach(fluxTable =>
        {
            var fluxRecords = fluxTable.Records;
            fluxRecords.ForEach(fluxRecord =>
            {
                //Console.WriteLine($"{fluxRecord.GetTime()}: {fluxRecord.GetValue()}");
            });
        });

        //throw new NotImplementedException();
    }
    protected TDto ToTDto(TModel model)
    {
        return model.ToDto() as TDto ?? throw new MappingException(typeof(TModel), typeof(TDto));
    }

    protected TModel ToTModel(TDto dto)
    {
        return dto.ToModel() as TModel ?? throw new MappingException(typeof(TDto), typeof(TModel));
    }
    //FromInfluxDataToModel
    protected TModel FromInfluxDataToModel(TDto dto, List<FluxTable>  fluxTables)
    {
        return dto.FromInfluxDataToModel(fluxTables) as TModel ?? throw new MappingException(typeof(TDto), typeof(TModel));
    }
}
