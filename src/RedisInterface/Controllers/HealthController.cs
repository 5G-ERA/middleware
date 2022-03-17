using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/[controller]")]
    [ApiController]
    public class HealthController : ControllerBase
    {
        private readonly HttpClient _client;

        public HealthController(IHttpClientFactory factory)
        {
            _client = factory.CreateClient("helthCheckClinet");
        }
        [HttpGet(Name="HelthCheck")]
        public IActionResult Get()
        {
            return Ok();
        }
        [HttpGet]
        [Route("spec", Name = "GetSpec")]
        public async Task<IActionResult> GetSpec()
        {
            string path = string.Empty;
#if DEBUG
            var addresses = Environment.GetEnvironmentVariable("ASPNETCORE_URLS")?.Split(";");
            if (addresses == null)
            {
                return NotFound();
            }
            _client.BaseAddress = new Uri(addresses[0]);

            var bytes = await _client.GetByteArrayAsync("/swagger/v1/swagger.json");
            path = Path.Combine(Directory.GetCurrentDirectory(), "RedisInterfaceSpec.json");
            System.IO.File.WriteAllBytes(path, bytes);
#endif
            return Ok(path);
        }
    }
}
