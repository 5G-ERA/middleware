using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace Middleware.Models.Domain.Slice
{
    public class UrllcSliceModel
    {
        [JsonPropertyName("SliceId")]
        public string SliceId { get; set; }

        [JsonPropertyName("Site")]
        public string Site { get; set; }

        [JsonPropertyName("Latency")]
        public int Latency { get; set; }

        [JsonPropertyName("Jitter")]
        public int Jitter { get; set; }

        [JsonPropertyName("ExpDataRateUl")]
        public int ExpDataRateUl { get; set; }

        [JsonPropertyName("ExpDataRateDl")]
        public int ExpDataRateDl { get; set; }

        [JsonPropertyName("TrafficType")]
        public string TrafficType { get; set; }

        [JsonPropertyName("Imsi")]
        public List<string> Imsi { get; set; }
    }
}
