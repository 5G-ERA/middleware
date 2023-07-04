using FluentAssertions;
using Middleware.Common.ExtensionMethods;

namespace Common.Tests.Unit;

public class ListExtensionTests
{
    [Fact]
    public void CreateCopy_ShouldCreateCopyThatHasTheSameElementsButIsNotSameInstance()
    {
        //arrange
        var list = new List<string>
        {
            "a", "ab", "abc", "abcde"
        };
        //act
        var result = list.CreateCopy();
        //assert
        result.Should().NotBeNull();
        result.Should().HaveCount(list.Count);
        result.Should().BeEquivalentTo(list);
        result.Should().NotBeSameAs(list);
    }
}