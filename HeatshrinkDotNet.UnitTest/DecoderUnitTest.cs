using System;

using HeatshrinkDotNet;

using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace HeatshrinkDotNet.UnitTest
{
    [TestClass]
    public class DecoderUnitTest
    {
        [TestMethod]
        [ExpectedException(typeof(ArgumentOutOfRangeException))]
        public void DecoderAllocShouldRejectExcessivelySmallWindow()
        {
            new HeatshrinkDecoder(256, Constants.MinWindowBits - 1, 4);
        }

        [TestMethod]
        [ExpectedException(typeof(ArgumentOutOfRangeException))]
        public void DecoderAllocShouldRejectZeroByteInputBuffer()
        {
            new HeatshrinkDecoder(0, Constants.MinWindowBits, Constants.MinWindowBits - 1);
        }

        [TestMethod]
        [ExpectedException(typeof(ArgumentOutOfRangeException))]
        public void DecoderAllocShouldRejectLookaheadEqualToWindowSize()
        {
            new HeatshrinkDecoder(0, Constants.MinWindowBits, Constants.MinWindowBits);
        }

        [TestMethod]
        [ExpectedException(typeof(ArgumentOutOfRangeException))]
        public void DecoderAllocShouldRejectLookaheadGreaterThanWindowSize()
        {
            new HeatshrinkDecoder(0, Constants.MinWindowBits, Constants.MinWindowBits + 1);
        }

        [TestMethod]
        public void DecoderSinkShouldRejectExcessivelyLargeInput()
        {
            var input = new byte[] { 0, 1, 2, 3, 4, 5 };
            var decoder = new HeatshrinkDecoder(1, Constants.MinWindowBits, Constants.MinWindowBits - 1);

            // Sink as much as will fit
            var res = decoder.Sink(input, out var count);
            Assert.AreEqual(DecoderSinkResult.Ok, res);
            Assert.AreEqual(1, count);

            // And now, no more should fit.
            res = decoder.Sink(input, count, input.Length - count, out count);
            Assert.AreEqual(DecoderSinkResult.Full, res);
            Assert.AreEqual(0, count);
        }

        [TestMethod]
        public void DecoderPollShouldReturnEmptyIfEmpty()
        {
            var output = new byte[256];
            var decoder = new HeatshrinkDecoder(256, Constants.MinWindowBits, Constants.MinWindowBits - 1);

            var res = decoder.Poll(output, out var outSz);
            Assert.AreEqual(DecoderPollResult.Empty, res);
        }

        [TestMethod]
        public void DecoderPollShouldExpandShortLiteral()
        {
            var input = new byte[] { 0xb3, 0x5b, 0xed, 0xe0 };
            var output = new byte[4];
            var decoder = new HeatshrinkDecoder(256, 7, 3);

            var sres = decoder.Sink(input, out var count);
            Assert.AreEqual(DecoderSinkResult.Ok, sres);

            var pres = decoder.Poll(output, out var outSz);
            Assert.AreEqual(DecoderPollResult.Empty, pres);
            Assert.AreEqual(3, outSz);
            Assert.AreEqual((byte)'f', output[0]);
            Assert.AreEqual((byte)'o', output[1]);
            Assert.AreEqual((byte)'o', output[2]);
        }

        [TestMethod]
        public void DecoderPollShouldExpandShortLiteralAndBackref()
        {
            var input = new byte[] { 0xb3, 0x5b, 0xed, 0xe0, 0x41, 0x00 }; // "foofoo"
            var output = new byte[6];
            var decoder = new HeatshrinkDecoder(256, 7, 6);

            var sres = decoder.Sink(input, out var count);
            Assert.AreEqual(DecoderSinkResult.Ok, sres);

            decoder.Poll(output, out var outSz);

            Assert.AreEqual(6, outSz);
            Assert.AreEqual((byte)'f', output[0]);
            Assert.AreEqual((byte)'o', output[1]);
            Assert.AreEqual((byte)'o', output[2]);
            Assert.AreEqual((byte)'f', output[3]);
            Assert.AreEqual((byte)'o', output[4]);
            Assert.AreEqual((byte)'o', output[5]);
        }

        [TestMethod]
        public void DecoderPollShouldExpandShortSelfOverlappingBackref()
        {
            // "aaaaa" == (literal, 1), ('a'), (backref, 1 back, 4 bytes)
            var input = new byte[] { 0xb0, 0x80, 0x01, 0x80 };
            var output = new byte[6];
            var expected = new byte[] { (byte)'a', (byte)'a', (byte)'a', (byte)'a', (byte)'a' };
            var decoder = new HeatshrinkDecoder(256, 8, 7);

            var sres = decoder.Sink(input, out var count);
            Assert.AreEqual(DecoderSinkResult.Ok, sres);

            decoder.Poll(output, out var outSz);
            Assert.AreEqual(expected.Length, outSz);
            for (int i = 0; i < expected.Length; ++i) Assert.AreEqual(expected[i], output[i]);
        }

        [TestMethod]
        public void DecoderPollShouldSuspendIfOutOfSpaceInOutputBufferDuringLiteralExpansion()
        {
            var input = new byte[] { 0xb3, 0x5b, 0xed, 0xe0, 0x40, 0x80 };
            var output = new byte[1];
            var decoder = new HeatshrinkDecoder(256, 7, 6);

            var sres = decoder.Sink(input, out var count);
            Assert.AreEqual(DecoderSinkResult.Ok, sres);

            var pres = decoder.Poll(output, out var outSz);
            Assert.AreEqual(DecoderPollResult.More, pres);
            Assert.AreEqual(1, outSz);
            Assert.AreEqual((byte)'f', output[0]);
        }

        [TestMethod]
        public void DecoderPollShouldSuspendIfOutOfSpaceInOutputBufferDuringBackrefExpansion()
        {
            var input = new byte[] { 0xb3, 0x5b, 0xed, 0xe0, 0x41, 0x00 }; // "foofoo"
            var output = new byte[4];
            var decoder = new HeatshrinkDecoder(256, 7, 6);

            var sres = decoder.Sink(input, out var count);
            Assert.AreEqual(DecoderSinkResult.Ok, sres);

            var pres = decoder.Poll(output, out var outSz);
            Assert.AreEqual(DecoderPollResult.More, pres);
            Assert.AreEqual(4, outSz);
            Assert.AreEqual((byte)'f', output[0]);
            Assert.AreEqual((byte)'o', output[1]);
            Assert.AreEqual((byte)'o', output[2]);
            Assert.AreEqual((byte)'f', output[3]);
        }

        [TestMethod]
        public void DecoderPollShouldExpandShortLiteralAndBackrefWhenFedInputByteByByte()
        {
            var input = new byte[] { 0xb3, 0x5b, 0xed, 0xe0, 0x41, 0x00 }; // "foofoo"
            var output = new byte[7];
            var decoder = new HeatshrinkDecoder(256, 7, 6);

            for (var i = 0; i < 6; ++i)
            {
                var sres = decoder.Sink(input, i, 1, out var count);
                Assert.AreEqual(DecoderSinkResult.Ok, sres);
            }

            var pres = decoder.Poll(output, out var outSz);
            Assert.AreEqual(DecoderPollResult.Empty, pres);
            Assert.AreEqual(6, outSz);
            Assert.AreEqual((byte)'f', output[0]);
            Assert.AreEqual((byte)'o', output[1]);
            Assert.AreEqual((byte)'o', output[2]);
            Assert.AreEqual((byte)'f', output[3]);
            Assert.AreEqual((byte)'o', output[4]);
            Assert.AreEqual((byte)'o', output[5]);
        }

        [TestMethod]
        public void DecoderFinishShouldNoteWhenDoen()
        {
            var input = new byte[] { 0xb3, 0x5b, 0xed, 0xe0, 0x41, 0x00 }; // "foofoo"
            var output = new byte[7];
            var decoder = new HeatshrinkDecoder(256, 7, 6);

            var sres = decoder.Sink(input, out var count);
            Assert.AreEqual(DecoderSinkResult.Ok, sres);

            var pres = decoder.Poll(output, out var outSz);
            Assert.AreEqual(DecoderPollResult.Empty, pres);
            Assert.AreEqual(6, outSz);
            Assert.AreEqual((byte)'f', output[0]);
            Assert.AreEqual((byte)'o', output[1]);
            Assert.AreEqual((byte)'o', output[2]);
            Assert.AreEqual((byte)'f', output[3]);
            Assert.AreEqual((byte)'o', output[4]);
            Assert.AreEqual((byte)'o', output[5]);

            var fres = decoder.Finish();
            Assert.AreEqual(DecoderFinishResult.Done, fres);
        }

        [TestMethod]
        public void DecoderShouldNotGetStuckWithFinishYieldingMoreButZeroBytesOutputFromPoll()
        {
            var input = new byte[512];
            for (var i = 0; i < 256; ++i) input[i] = 0xff;

            var decoder = new HeatshrinkDecoder(256, 8, 4);

            /* Confirm that no byte of trailing context can lead to
             * heatshrink_decoder_finish erroneously returning HSDR_FINISH_MORE
             * when heatshrink_decoder_poll will yield 0 bytes.
             *
             * Before 0.3.1, a final byte of 0xFF could potentially cause
             * this to happen, if at exactly the byte boundary. */
            for (int b = 0; b < 256; ++b)
            {
                for (int i = 1; i < 512; ++i)
                {
                    input[i] = (byte)b;
                    decoder.Reset();

                    var output = new byte[1024];
                    var sres = decoder.Sink(input, out var count);
                    Assert.AreEqual(DecoderSinkResult.Ok, sres);

                    var pres = decoder.Poll(output, out var outSz);
                    Assert.AreEqual(DecoderPollResult.Empty, pres);

                    var fres = decoder.Finish();
                    Assert.AreEqual(DecoderFinishResult.Done, fres);

                    input[i] = 0xff;
                }
            }
        }
    }
}
