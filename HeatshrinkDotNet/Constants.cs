using System;

namespace HeatshrinkDotNet
{
    public static class Constants
    {
        public const bool EnableLogging = false;

        public const int MinWindowBits = 4;
        public const int MaxWindowBits = 15;

        public const int MinLookaheadBits = 3;

        public const byte LiteralMarker = 0x01;
        public const byte BackrefMarker = 0x00;
    }
}
