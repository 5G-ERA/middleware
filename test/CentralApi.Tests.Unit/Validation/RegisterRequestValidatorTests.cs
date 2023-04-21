using FluentValidation.TestHelper;
using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Validation;

namespace CentralApi.Tests.Unit.Validation;


public class RegisterRequestValidatorTests
{
    private readonly RegisterRequestValidator _sut;

    public RegisterRequestValidatorTests()
    {
        _sut = new RegisterRequestValidator();
    }

    [Fact]
    public void RegisterRequestValidator_ShouldHaveError_WhenLocationNameIsNull()
    {
        var model = new RegisterRequest { Name = null };
        var result = _sut.TestValidate(model);
        result.ShouldHaveValidationErrorFor(request => request.Name);
    }
    [Fact]
    public void RegisterRequestValidator_ShouldHaveError_WhenLocationNameIsEmpty()
    {
        var model = new RegisterRequest { Name = string.Empty };
        var result = _sut.TestValidate(model);
        result.ShouldHaveValidationErrorFor(request => request.Name);
    }
    
    [Fact]
    public void RegisterRequestValidator_ShouldHaveError_WhenLocationOrganizationIsNull()
    {
        var model = new RegisterRequest { Organization = null };
        var result = _sut.TestValidate(model);
        result.ShouldHaveValidationErrorFor(request => request.Organization);
    }
    [Fact]
    public void RegisterRequestValidator_ShouldHaveError_WhenLocationOrganizationIsEmpty()
    {
        var model = new RegisterRequest { Organization = string.Empty };
        var result = _sut.TestValidate(model);
        result.ShouldHaveValidationErrorFor(request => request.Organization);
    }
    
    [Fact]
    public void RegisterRequestValidator_ShouldHaveError_WhenLocationTypeIsNull()
    {
        var model = new RegisterRequest { Type = null };
        var result = _sut.TestValidate(model);
        result.ShouldHaveValidationErrorFor(request => request.Type);
    }
    [Fact]
    public void RegisterRequestValidator_ShouldHaveError_WhenLocationTypeDoesNotContainRequiredValue()
    {
        var model = new RegisterRequest { Type = "Definitely not the correctType" };
        var result = _sut.TestValidate(model);
        result.ShouldHaveValidationErrorFor(request => request.Type);
    }
    
    [Fact]
    public void RegisterRequestValidator_ShouldNotHaveError_WhenLocationTypeIsCloud()
    {
        var model = new RegisterRequest { Type = "Cloud" };
        var result = _sut.TestValidate(model);
        result.ShouldNotHaveValidationErrorFor(request => request.Type);
    }
    [Fact]
    public void RegisterRequestValidator_ShouldNotHaveError_WhenLocationTypeIsEdge()
    {
        var model = new RegisterRequest { Type = "Cloud" };
        var result = _sut.TestValidate(model);
        result.ShouldNotHaveValidationErrorFor(request => request.Type);
    }
    
    [Fact]
    public void RegisterRequestValidator_ShouldNotHaveError_WhenLocationTypeIsCorrectAndCaseInsensitive()
    {
        var model = new RegisterRequest { Type = "ClOUd" };
        var result = _sut.TestValidate(model);
        result.ShouldNotHaveValidationErrorFor(request => request.Type);
    }
    
    [Fact]
    public void RegisterRequestValidator_ShouldNotHaveError_WhenCorrectLocationIsSpecified()
    {
        var model = new RegisterRequest
        {
            Name = "TestCloud",
            Organization = "TestOrganization",
            Type = "Cloud"
        };
        var result = _sut.TestValidate(model);
        result.ShouldNotHaveValidationErrorFor(request => request.Type);
    }
}