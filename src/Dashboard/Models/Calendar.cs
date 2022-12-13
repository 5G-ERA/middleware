using Microsoft.AspNetCore.Mvc;

namespace Dashboard.Models
{
    public class Test
    {
        [BindProperty]
        public DateTime? RegisterDate { get; set; }

    }
}
