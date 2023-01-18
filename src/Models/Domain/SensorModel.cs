namespace Middleware.Models.Domain
{
    public class SensorModel
    {
        public string Name { get; set; }
        public string Type { get; set; }
        // public string SensorLocation  { get; set; }
        public string Description { get; set; }
        public List<string> Nodes { get; set; } //A sensor can publish multiple topics

        public int number { get; set; }
    }
}