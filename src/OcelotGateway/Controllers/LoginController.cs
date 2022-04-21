using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Repositories;
using Middleware.Common.Models;
using Middleware.OcelotGateway.Services;
using Middleware.Common.Repositories.Abstract;
using System;
using System.Security.Cryptography;
using Microsoft.AspNetCore.Cryptography.KeyDerivation;

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
                byte[] salt = new byte[128 / 8];
                using (var rngCsp = new RNGCryptoServiceProvider())
                {
                    rngCsp.GetNonZeroBytes(salt);
                }
                register.Salt = Convert.ToBase64String(salt);
                string hashedPassword = Convert.ToBase64String(KeyDerivation.Pbkdf2(
                password: register.Password,
                salt: salt,
                prf: KeyDerivationPrf.HMACSHA256,
                iterationCount: 100000,
                numBytesRequested: 256 / 8));
                register.Password = hashedPassword;
        
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
            string computedHashedPassword = Convert.ToBase64String(KeyDerivation.Pbkdf2(
                password: login.Password,
                salt: salt,
                prf: KeyDerivationPrf.HMACSHA256,
                iterationCount: 100000,
                numBytesRequested: 256 / 8));
            return (computedHashedPassword.Equals(storedCredentials.Password)) ? true : false;


            /*if (login.Id.ToString() == "1e309941-4cbb-47e9-98ef-c2e38b5f157b") 
            {
                user = new UserModel() { Id = Guid.Parse("1e309941-4cbb-47e9-98ef-c2e38b5f157b"), Password = "password" };
            }
            return user;*/
            //return user;
        }
    }
}
