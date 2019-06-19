using System;
using System.Collections.Generic;
using System.Text;

namespace HeatshrinkDotNet
{
    static class Logging
    {
        internal static void Write(string msg)
        {
            if (Constants.EnableLogging) Console.Write(msg);
        }

        internal static void WriteLine(string msg)
        {
            if (Constants.EnableLogging) Console.WriteLine(msg);
        }
    }
}
