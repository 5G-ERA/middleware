using FluentAssertions;
using Middleware.Common;

namespace Common.Tests.Unit;

public class K8SImageHelperTests
{

    [Theory]
    [InlineData("", "nginx", "", "nginx")]
    [InlineData("dev", "nginx", "", "dev/nginx")]
    [InlineData("dev", "nginx", "1.2.8", "dev/nginx:1.2.8")]
    [InlineData("", "nginx", "1.2.8", "nginx:1.2.8")]
    public void BuildImageName_ShouldCombineValues_WhenValuesAreProvided(string registry, string repositoryName,
        string tag, string expected)
    {
        // arrange
        // act
        string imageName = K8SImageHelper.BuildImageName(registry, repositoryName, tag);
        // assess
        Assert.Equal(expected, imageName);
    }

    [Fact]
    public void BuildImageName_ShouldThrowException_WhenRepositoryNameNotSpecified()
    {
        // arrange
        var repositoryName = string.Empty;
        // act
        var func = () => K8SImageHelper.BuildImageName(null, repositoryName, null);

        // assess
        func.Should().Throw<ArgumentException>().WithMessage("Repository name not provided. (Parameter 'repositoryName')").WithParameterName(nameof(repositoryName));
    }

    [Theory]
    [InlineData("redmine:latest", "latest")]
    [InlineData("orchestrator", "latest")]
    [InlineData("orchestrator:v0.1.3", "v0.1.3")]

    public void GetTag_ShouldReturnCorrectTag(string image, string expected)
    {
        // arrange
        // act
        string tag = K8SImageHelper.GetTag(image);
        //assess
        Assert.Equal(expected, tag);
    }
}
