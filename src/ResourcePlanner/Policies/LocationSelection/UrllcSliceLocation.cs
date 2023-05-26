using Middleware.Models.Domain;
using Middleware.Models.Enums;

namespace Middleware.ResourcePlanner.Policies.LocationSelection;

internal class UrllcSliceLocation : ILocationSelectionPolicy
{
    public UrllcSliceLocation(Priority priority)
    {
        Priority = priority;
    }

    /// <inheritdoc />
    public Priority Priority { get; }

    /// <inheritdoc />
    public Task<PlannedLocation> GetLocationAsync()
    {
        //TODO:
        /**
         * 1. Znajdź lokacje mające dostęp do Urrlc
         *  a. jak to zapisać oraz odczytać będzie musiało zostać rozwiązane w ramamch #78
         * 2. Wybierz najbliższą danemu robotowi
         *  a. Relacja na grafie z pingiem między robotem a dostępnymi lokacjami
         * 3. Zwróć dane o lokacji
         */
        throw new NotImplementedException();
    }

    /// <inheritdoc />
    public Task<bool> IsLocationSatisfiedByPolicy(PlannedLocation plannedLocation)
    {
        throw new NotImplementedException();
    }
}