using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Repositories;
using Middleware.Common.Models;
using Middleware.OcelotGateway.Services;
using Middleware.Common.Repositories.Abstract;

namespace Middleware.OcelotGateway.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class LoginController : ControllerBase
    {
        private readonly IUserRepository _userRepository;
        private readonly ILogger _logger;

        public LoginController(IUserRepository userRepository, ILogger logger)
        {
            _userRepository = userRepository ?? throw new ArgumentNullException(nameof(userRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }



        [AllowAnonymous]
        [HttpPost(Name = "Register")]
        public IActionResult Register([FromBody] UserModel register)
        {
            UserModel user = new UserModel();
            return (IActionResult)user;
        }


        [AllowAnonymous]
        [HttpPost(Name = "Login")]
        public IActionResult Login([FromBody] UserModel login) 
        {
            IActionResult response = Unauthorized();
            
            UserModel user = AuthenticateUser(login);
            if (user != null)
            {
                TokenService token = new TokenService();
                var newToken = token.GenerateToken(user);
                response = Ok(newToken);
            }
            return response;
        }


        private  UserModel AuthenticateUser(UserModel login) 
        {
            UserModel user = null;
            if (login.Id.ToString() == "1e309941-4cbb-47e9-98ef-c2e38b5f157b") 
            {
                user = new UserModel() { Id = Guid.Parse("1e309941-4cbb-47e9-98ef-c2e38b5f157b"), Password = "password" };
            }
            return user;
        }
    }
}
