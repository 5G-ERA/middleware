using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Repositories;
using Middleware.Common.Models;
using Middleware.OcelotGateway.Services;
using Middleware.Common.Repositories.Abstract;

namespace Middleware.OcelotGateway.Controllers
{
    [Route("api/v1")]
    [ApiController]
    public class LoginController : ControllerBase
    {
        private readonly IUserRepository _userRepository;
        private readonly ILogger _logger;

        public LoginController(IUserRepository userRepository, ILogger<LoginController> logger)
        {
            _userRepository = userRepository ?? throw new ArgumentNullException(nameof(userRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }



        [AllowAnonymous]
        [HttpPost]
        [Route("register", Name = "Register")]
        public async Task<ActionResult<UserModel>> Register([FromBody] UserModel register)
        {
            if (register == null)
            {
                BadRequest("Please enter valid credentials");
            }
            try
            {
                register.Salt = "saltstring";
                await _userRepository.AddAsync(register);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
            }
            return Ok(register);
        }


        [AllowAnonymous]
        [HttpPost]
        [Route("login", Name = "Login")]
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
