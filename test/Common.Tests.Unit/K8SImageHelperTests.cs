using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Middleware.Common;

namespace Common.Tests.Unit;

public class K8SImageHelperTests
{

    [Theory]
    [InlineData("redmine:latest", "latest")]
    [InlineData("orchestrator", "latest")]
    [InlineData("orchestrator:v0.1.3", "v0.1.3")]

    public void GetTag_ShouldReturnLatest_WhenImageIsLatest(string image, string expected)
    {
        // arrange
        // act
        string tag = K8SImageHelper.GetTag(image);
        //assess
        Assert.Equal(expected, tag);
    }
}