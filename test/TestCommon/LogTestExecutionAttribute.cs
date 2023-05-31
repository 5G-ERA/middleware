using System.Collections.Concurrent;
using System.Reflection;
using Xunit.Sdk;

namespace TestCommon;

public class LogTestExecutionAttribute : BeforeAfterTestAttribute
{
    public override void Before(MethodInfo methodUnderTest)
    {
        TestExecutionDataLogger.LogBegin(methodUnderTest);
    }

    public override void After(MethodInfo methodUnderTest)
    {
        TestExecutionDataLogger.LogEnd(methodUnderTest);
    }
}

public static class TestExecutionDataLogger
{
    private static readonly string LogFileName =
        Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "Middleware",
            $"UnitTests_{DateTime.UtcNow:yyyy_MM_dd_HH_mm}_D_{AppDomain.CurrentDomain.Id}.csv");

    private static int _startedOrder;
    private static int _endedOrder;
    private static readonly ConcurrentDictionary<string, TestExecutionData> testDataDict = new();
    private static readonly ConcurrentQueue<string> logQueue = new();

    public static void LogBegin(MethodInfo testInfo)
    {
        var name = $"{testInfo.DeclaringType.FullName}.{testInfo.Name}";
        var order = Interlocked.Add(ref _startedOrder, 1);
        var startedUtc = DateTime.UtcNow;
        var data = testDataDict.GetOrAdd(name, new TestExecutionData());
        data.StartedUtc = startedUtc;
        data.StartedOrder = order;
        data.TestName = name;
        data.Status = "Started";
        data.StartThreadId = Thread.CurrentThread.ManagedThreadId;
        WriteLog(data);
    }

    public static void LogEnd(MethodInfo testInfo)
    {
        var name = $"{testInfo.DeclaringType.FullName}.{testInfo.Name}";
        var dataEndedUtc = DateTime.UtcNow;
        var order = Interlocked.Add(ref _endedOrder, 1);
        var data = testDataDict[name];
        data.EndedUtc = dataEndedUtc;
        data.EndedOrder = order;
        data.Status = "Ended";
        data.EndThreadId = Thread.CurrentThread.ManagedThreadId;
        WriteLog(data);
    }

    private static void WriteLog(TestExecutionData data)
    {
        logQueue.Enqueue(data.ToCsvLine());

        if (data.EndedOrder == 1)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(LogFileName));
            Task.Run(LogWriter);
        }
    }

    private static Task LogWriter()
    {
        while (true)
        {
            var logs = new List<string>();
            string result;
            while (logQueue.TryDequeue(out result))
            {
                logs.Add(result);
            }

            if (logs.Any()) File.AppendAllLines(LogFileName, logs);
        }
    }

    private class TestExecutionData
    {
        public int StartedOrder { get; set; }
        public int EndedOrder { get; set; }
        public DateTime StartedUtc { get; set; }
        public DateTime EndedUtc { get; set; }
        public string TestName { get; set; } = null!;
        public string Status { get; set; } = null!;
        public int StartThreadId { get; set; }
        public int EndThreadId { get; set; }

        public string ToCsvLine()
        {
            return
                $"{TestName};{Status};{StartedOrder};{EndedOrder};{StartedUtc:o};{EndedUtc:o};{Math.Max(0, (EndedUtc - StartedUtc).TotalMilliseconds)};{StartThreadId};{EndThreadId}";
        }
    }
}