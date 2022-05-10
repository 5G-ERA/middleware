using System.Net;
using System.Security.Cryptography;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Cryptography.KeyDerivation;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories.Abstract;
using Middleware.OcelotGateway.Services;

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
                byte[] salt = RandomNumberGenerator.GetBytes(128 / 8);
                register.Salt = Convert.ToBase64String(salt);

                register.Password = ComputeHashPassword(register.Password, salt);
        
                await _userRepository.AddAsync(register);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
            }
            return StatusCode((int)HttpStatusCode.Created);
        }


        [AllowAnonymous]
        [HttpPost]
        [Route("login", Name = "Login")]
        public async Task<IActionResult> Login([FromBody] UserModel login) 
        {
            IActionResult response = Unauthorized();
            
            bool authenticated = await AuthenticateUser(login);
            if (authenticated)
            {
                TokenService token = new TokenService();
                var newToken = token.GenerateToken(login.Id);
                response = Ok(newToken);
            }
            return response;
        }


        private async Task<bool> AuthenticateUser(UserModel login) 
        {
            UserModel storedCredentials = await _userRepository.GetByIdAsync(login.Id);
            byte[] salt = Convert.FromBase64String(storedCredentials.Salt);
            
            string computedHashedPassword = ComputeHashPassword(login.Password, salt);

            return computedHashedPassword.Equals(storedCredentials.Password) ? true : false; 
        }


        private string ComputeHashPassword(string clearTextPassword, byte[] salt) 
        {
            string hashedPassword = Convert.ToBase64String(KeyDerivation.Pbkdf2(
                password: clearTextPassword,
                salt: salt,
                prf: KeyDerivationPrf.HMACSHA256,
                iterationCount: 100000,
                numBytesRequested: 256 / 8));
            return hashedPassword;
        }
    }
}
