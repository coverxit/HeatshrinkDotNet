using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace HeatshrinkDotNet.UnitTest
{
    [TestClass]
    public class RegressionUnitTest
    {
        [TestMethod]
        public void SmallInputBufferShouldNotImpactDecoderCorrectness()
        {
            var input = new byte[5];
            var cfg = new ConfigInfo
            {
                LogLevel = 0,
                WindowSz = 8,
                LookaheadSz = 3,
                DecoderInputBufferSize = 5
            };
            for (int i = 0; i < 5; ++i) input[i] = (byte)('a' + (i % 26));
            Helper.CompressAndExpandAndCheck(input, cfg);
        }

        [TestMethod]
        public void RegressionBackreferenceCountersShouldNotRollover()
        {
            /* Searching was scanning the entire context buffer, not just
             * the maximum range addressable by the backref index.*/

            var input = new byte[337];
            Helper.FillWithPseudoRandomLetters(input, 3);

            var cfg = new ConfigInfo
            {
                LogLevel = 3,
                WindowSz = 8,
                LookaheadSz = 3,
                DecoderInputBufferSize = 64 // 1
            };
            Helper.CompressAndExpandAndCheck(input, cfg);
        }

        [TestMethod]
        public void RegressionIndexFail()
        {
            /* Failured when indexed, cause unknown.
             *
             * This has something to do with bad data at the very last
             * byte being indexed, due to spillover. */

            var input = new byte[507];
            Helper.FillWithPseudoRandomLetters(input, 3);

            var cfg = new ConfigInfo
            {
                LogLevel = 0,
                WindowSz = 8,
                LookaheadSz = 3,
                DecoderInputBufferSize = 64
            };
            Helper.CompressAndExpandAndCheck(input, cfg);
        }

        [TestMethod]
        public void SixtyFourK()
        {
            /* Regression: An input buffer of 64k should not cause an
             * overflow that leads to an infinite loop. */

            var input = new byte[64 * 1024];
            Helper.FillWithPseudoRandomLetters(input, 1);

            var cfg = new ConfigInfo
            {
                LogLevel = 0,
                WindowSz = 8,
                LookaheadSz = 3,
                DecoderInputBufferSize = 64
            };
            Helper.CompressAndExpandAndCheck(input, cfg);
        }
    }
}
