using System;
using System.Linq;

using HeatshrinkDotNet;

using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace HeatshrinkDotNet.UnitTest
{
    [TestClass]
    public class EncoderUnitTest
    {
        [TestMethod]
        public void EncoderAllocShouldRejectInvalidArguments()
        {
            void tester(byte windowBits, byte lookaheadBits)
            {
                try
                {
                    new HeatshrinkEncoder(windowBits, lookaheadBits);
                    Assert.Fail();
                }
                catch (ArgumentOutOfRangeException)
                {
                    // Excpected
                }
            }

            tester(Constants.MinWindowBits - 1, 8);
            tester(Constants.MaxWindowBits + 1, 8);
            tester(8, Constants.MinLookaheadBits - 1);
            tester(8, 9);
        }

        [TestMethod]
        public void EncoderSinkShouldAcceptInputWhenItWillFit()
        {
            var encoder = new HeatshrinkEncoder(8, 7);
            var input = Enumerable.Repeat('*', 256).Select(c => (byte)c).ToArray();

            Assert.AreEqual(EncoderSinkResult.Ok, encoder.Sink(input, out var bytesCopied));
            Assert.AreEqual(256, bytesCopied);
        }

        [TestMethod]
        public void EncoderSinkShouldAcceptPartialInputWhenSomeWillFit()
        {
            var encoder = new HeatshrinkEncoder(8, 7);
            var input = Enumerable.Repeat('*', 512).Select(c => (byte)c).ToArray();

            Assert.AreEqual(EncoderSinkResult.Ok, encoder.Sink(input, out var bytesCopied));
            Assert.AreEqual(256, bytesCopied);
        }

        [TestMethod]
        public void EncoderPollShouldIndicateWhenNoInputIsProvided()
        {
            var encoder = new HeatshrinkEncoder(8, 7);
            var output = new byte[512];

            Assert.AreEqual(EncoderPollResult.Empty, encoder.Poll(output, out var outputSize));
        }

        [TestMethod]
        public void EncoderShouldEmitDataWithoutRepetitionsAsLiteralSequence()
        {
            var encoder = new HeatshrinkEncoder(8, 7);

            var input = new byte[5];
            var output = new byte[1024];
            var expected = new byte[] { 0x80, 0x40, 0x60, 0x50, 0x38, 0x20 };

            for (int i = 0; i < 5; ++i) input[i] = (byte)i;

            Assert.AreEqual(EncoderSinkResult.Ok, encoder.Sink(input, out var copied));
            Assert.AreEqual(5, copied);

            // Should get no output yet, since encoder doesn't know input is complete. 
            var pres = encoder.Poll(output, out copied);
            Assert.AreEqual(EncoderPollResult.Empty, pres);
            Assert.AreEqual(0, copied);

            // Mark input stream as done, to force small input to be processed.
            var fres = encoder.Finish();
            Assert.AreEqual(EncoderFinishResult.More, fres);

            pres = encoder.Poll(output, out copied);
            Assert.AreEqual(EncoderPollResult.Empty, pres);

            for (int i = 0; i < expected.Length; ++i)
                Assert.AreEqual(expected[i], output[i]);

            Assert.AreEqual(EncoderFinishResult.Done, encoder.Finish());
        }

        [TestMethod]
        public void EncoderShouldEmitSeriesOfSameByteAsLiteralThenBackRef()
        {
            var encoder = new HeatshrinkEncoder(8, 7);

            var input = new byte[5];
            var output = new byte[1024];
            var expected = new byte[] { 0xb0, 0x80, 0x01, 0x80 };

            for (int i = 0; i < 5; ++i) input[i] = (byte)'a'; // "aaaaa";

            Assert.AreEqual(EncoderSinkResult.Ok, encoder.Sink(input, out var copied));
            Assert.AreEqual(5, copied);

            // Should get no output yet, since encoder doesn't know input is complete.
            var pres = encoder.Poll(output, out copied);
            Assert.AreEqual(EncoderPollResult.Empty, pres);
            Assert.AreEqual(0, copied);

            // Mark input stream as done, to force small input to be processed. 
            var fres = encoder.Finish();
            Assert.AreEqual(EncoderFinishResult.More, fres);

            pres = encoder.Poll(output, out copied);
            Assert.AreEqual(EncoderPollResult.Empty, pres);
            Assert.AreEqual(4, copied);

            for (int i = 0; i < copied; ++i) Assert.AreEqual(expected[i], output[i]);

            Assert.AreEqual(EncoderFinishResult.Done, encoder.Finish());
        }

        [TestMethod]
        public void EncoderPollShouldDetectRepeatedSubstring()
        {
            var encoder = new HeatshrinkEncoder(8, 3);
            var input = new byte[] { (byte)'a', (byte)'b', (byte)'c', (byte)'d', (byte)'a', (byte)'b', (byte)'c', (byte)'d' };
            var output = new byte[1024];
            var expected = new byte[] { 0xb0, 0xd8, 0xac, 0x76, 0x40, 0x1b };

            var sres = encoder.Sink(input, out var copied);
            Assert.AreEqual(EncoderSinkResult.Ok, sres);
            Assert.AreEqual(input.Length, copied);

            var fres = encoder.Finish();
            Assert.AreEqual(EncoderFinishResult.More, fres);

            Assert.AreEqual(EncoderPollResult.Empty, encoder.Poll(output, out copied));
            fres = encoder.Finish();
            Assert.AreEqual(EncoderFinishResult.Done, fres);

            Assert.AreEqual(expected.Length, copied);
            for (int i = 0; i < expected.Length; ++i) Assert.AreEqual(expected[i], output[i]);
        }

        [TestMethod]
        public void EncoderPollShouldDetectRepeatedSubstringAndPreserveTrailingLiteral()
        {
            var encoder = new HeatshrinkEncoder(8, 3);
            var input = new byte[] { (byte)'a', (byte)'b', (byte)'c', (byte)'d', (byte)'a', (byte)'b', (byte)'c', (byte)'d', (byte)'e' };
            var output = new byte[1024];
            var expected = new byte[] { 0xb0, 0xd8, 0xac, 0x76, 0x40, 0x1b, 0xb2, 0x80 };

            var sres = encoder.Sink(input, out var copied);
            Assert.AreEqual(EncoderSinkResult.Ok, sres);
            Assert.AreEqual(input.Length, copied);

            var fres = encoder.Finish();
            Assert.AreEqual(EncoderFinishResult.More, fres);

            Assert.AreEqual(EncoderPollResult.Empty, encoder.Poll(output, out copied));
            fres = encoder.Finish();
            Assert.AreEqual(EncoderFinishResult.Done, fres);

            Assert.AreEqual(expected.Length, copied);
            for (int i = 0; i < expected.Length; ++i) Assert.AreEqual(expected[i], output[i]);
        }

        [TestMethod]
        public void Gen()
        {
            var encoder = new HeatshrinkEncoder(8, 7);
            var input = new byte[] { (byte)'a', (byte)'a', (byte)'a', (byte)'a', (byte)'a' };
            var output = new byte[1024];

            var sres = encoder.Sink(input, out var copied);
            Assert.AreEqual(EncoderSinkResult.Ok, sres);
            Assert.AreEqual(input.Length, copied);

            var fres = encoder.Finish();
            Assert.AreEqual(EncoderFinishResult.More, fres);

            Assert.AreEqual(EncoderPollResult.Empty, encoder.Poll(output, out copied));
            fres = encoder.Finish();
            Assert.AreEqual(EncoderFinishResult.Done, fres);
        }
    }
}
