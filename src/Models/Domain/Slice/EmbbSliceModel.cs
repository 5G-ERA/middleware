using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace Middleware.Models.Domain.Slice
{
    public class EmbbSliceModel
    {
        [JsonPropertyName("SliceId")]
        public string SliceId { get; set; }

        [JsonPropertyName("Site")]
        public string Site { get; set; }

        [JsonPropertyName("ExpDataRateUl")]
        public int ExpDataRateUl { get; set; }

        [JsonPropertyName("ExpDataRateDl")]
        public int ExpDataRateDl { get; set; }

        [JsonPropertyName("AreaTrafficCapUl")]
        public int AreaTrafficCapUl { get; set; }

        [JsonPropertyName("AreaTrafficCapDl")]
        public int AreaTrafficCapDl { get; set; }

        [JsonPropertyName("UserDensity")]
        public int UserDensity { get; set; }

        [JsonPropertyName("UserSpeed")]
        public int UserSpeed { get; set; }

        [JsonPropertyName("TrafficType")]
        public string TrafficType { get; set; }

        [JsonPropertyName("Imsi")]
        public List<string> Imsi { get; set; }
    }
}
