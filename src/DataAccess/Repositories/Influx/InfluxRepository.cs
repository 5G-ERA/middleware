﻿using InfluxDB.Client;
using InfluxDB.Client.Api.Domain;
using InfluxDB.Client.Core.Flux.Domain;
using InfluxDB.Client.Writes;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Exceptions;
using ILogger = Serilog.ILogger;


namespace Middleware.DataAccess.Repositories.Influx;
public class InfluxRepository<TModel, TDto> : IInfluxRepository<TModel, TDto> where TModel : BaseModel where TDto : InfluxDto, new()
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
        Bucket = bucket ?? throw new ArgumentNullException(nameof(bucket));
        ObjectType = objectType ?? throw new ArgumentNullException(nameof(objectType));
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
        /*
        point = PointData.Measurement("heartbeat")
            .Tag("robot", "robot55")
            .Tag("id", "3fa85f64-5717-4562-b3fc-2c963f66afa6")
            .Field("cpu", 55)
            .Field("ram", 2584)
            .Timestamp(DateTime.UtcNow, WritePrecision.Ns);
        //*/
        await Client.GetWriteApiAsync().WritePointAsync(point: point, bucket: bucket, org: org);
    }

    /// <summary>
    ///     Add to influx a sample from model.
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    public async Task<TModel?> AddAsync(TModel model)
    {
        var dto = ToTDto(model);
        var point = FromDtoToInflux(dto);
        await Client.GetWriteApiAsync().WritePointAsync(point: point, bucket: Bucket, org: Organization);
        return ToTModel(dto);
    }
    private async Task<List<String>> GetAllIdAsync()
    {        
        string query = "from(bucket: \"" + Bucket + "\") |> range(start: 0) |> keyValues(keyColumns: [\"Id\"]) |> keep(columns: [\"Id\"]) |> limit(n:1, offset: 0) |> group() ";
        var fluxTables = await Client.GetQueryApi().QueryAsync(query: query, org: Organization);
        List<String> ids = new List<string>();

        foreach (var record in fluxTables[0].Records)
        {
            var id = Guid.Empty.ToString();            
            var id2 = record.GetValueByKey("Id").ToString();

            if (id2 != null && id2 != "") id = id2;
            ids.Add(id);
        }
        return ids;
    }
    private async Task<List<TModel>> GetAllStatusByIdAsync(Guid id)
    {
        var stringId = id.ToString();
        string query = "from(bucket: \"" + Bucket + "\") |> range(start: 0) |> filter(fn: (r) => r[\"_measurement\"] == \"Heartbeat\") |> filter(fn: (r) => r[\"Id\"] == \"" + stringId + "\") |> yield(name: \"all\")";
        List<FluxTable> fluxTables = await Client.GetQueryApi().QueryAsync(query: query, org: Organization);

        List<TModel> listOfTModels = new();
        listOfTModels = AppendToListOfTModels(listOfTModels, fluxTables);
        return listOfTModels;
    }
    public async Task<List<TModel>> GetAllOneByOneAsync()
    {
        List<TModel> listOfTModels = new();
        var ids = await GetAllIdAsync();
        foreach (var id  in ids)
        {
            var guidId= Guid.Parse(id.ToString());
            try
            {
                List<TModel> listOfTModels2 = new();
                listOfTModels2 = await GetAllStatusByIdAsync(guidId);
                listOfTModels.AddRange(listOfTModels2);
            } catch (Exception ex)
            {
                //return listOfTModels;
            }            
        }
        return listOfTModels;
    }

    public async Task<TModel?> GetStatusByIdAsync(Guid id)
    {
        var stringId = id.ToString();
        string query = "from(bucket: \"" + Bucket + "\") |> range(start: 0) |> filter(fn: (r) => r[\"_measurement\"] == \"Heartbeat\") |> filter(fn: (r) => r[\"Id\"] == \"" + stringId + "\") |> yield(name: \"last\")";
        List<FluxTable> fluxTables = await Client.GetQueryApi().QueryAsync(query: query, org: Organization);
        if (fluxTables.Count > 0)
        {
            var dto = FromInfluxDataToDto(new TDto(), fluxTables);
            return ToTModel(dto);
        }
        else return null;
    }

    public async Task<List<TModel>> GetAllAsync()
    {
        var query = "from(bucket: \"" + Bucket + "\") |> range(start: 0) |> filter(fn: (r) => r[\"_measurement\"] == \"Heartbeat\")";
        List<FluxTable> allFluxTables = await Client.GetQueryApi().QueryAsync(query: query, org: Organization);

        List<FluxTable> oneObjectAllRecordsFluxTables = new();
        List<TModel> listOfTModels = new();

        var currentId = allFluxTables[0].Records[0].GetValueByKey("Id").ToString();
        foreach (var fluxTable in allFluxTables)
        {
            if (fluxTable.Records.Count > 0)
            {
                if (currentId == fluxTable.Records[0].GetValueByKey("Id").ToString())
                {
                    oneObjectAllRecordsFluxTables.Add(fluxTable);
                }
                else
                {
                    listOfTModels = AppendToListOfTModels(listOfTModels, oneObjectAllRecordsFluxTables);
                    oneObjectAllRecordsFluxTables.Clear();
                    currentId = fluxTable.Records[0].GetValueByKey("Id").ToString();
                    oneObjectAllRecordsFluxTables.Add(fluxTable);
                }
            }
        }
        listOfTModels = AppendToListOfTModels(listOfTModels, oneObjectAllRecordsFluxTables);
        oneObjectAllRecordsFluxTables.Clear();
        return listOfTModels;
    }
    private List<TModel> AppendToListOfTModels(List<TModel> listOfTModels, List<FluxTable> oneObjectAllRecordsFluxTables)
    {
        var currentNrRecords = oneObjectAllRecordsFluxTables[0].Records.Count;
        if(currentNrRecords < 1) return listOfTModels;

        for (var j = 0; j < currentNrRecords; j++)
        {
            var oneSampleAsFluxTables = GetOneSampleAsFluxTables(oneObjectAllRecordsFluxTables, recordNr: j);
            var dto = FromInfluxDataToDto(new TDto(), oneSampleAsFluxTables);
            var toTModel = ToTModel(dto);
            listOfTModels.Add(toTModel);
        }
        return listOfTModels;
    }

    // Asumption that nr of records in each Measurement (field) is the same for everyone in given object
    private List<FluxTable> GetOneSampleAsFluxTables(List<FluxTable> oneObjectAllRecordsFluxTables, int recordNr)
    {
        List<FluxTable> oneSampleAsFluxTables = new();
        foreach (var fluxTable in oneObjectAllRecordsFluxTables)
        {
            var record = fluxTable.Records[recordNr];
            FluxTable oneRecordAsFluxTable = new();
            oneRecordAsFluxTable.Records.Add(record);
            oneSampleAsFluxTables.Add(oneRecordAsFluxTable);
        }
        return oneSampleAsFluxTables;
    }
    protected TDto ToTDto(TModel model)
    {
        return model.ToDto() as TDto ?? throw new MappingException(typeof(TModel), typeof(TDto));
    }

    protected TModel ToTModel(TDto dto)
    {
        return dto.ToModel() as TModel ?? throw new MappingException(typeof(TDto), typeof(TModel));
    }
    protected TDto FromInfluxDataToDto(TDto dto, List<FluxTable> fluxTables)
    {
        return dto.FromInfluxDataToDto(fluxTables) as TDto ?? throw new MappingException(typeof(FluxTable), typeof(TDto));
    }
    protected PointData FromDtoToInflux(TDto dto)
    {
        return dto.ToPointData() ?? throw new MappingException(typeof(TDto), typeof(PointData));
    }
}
