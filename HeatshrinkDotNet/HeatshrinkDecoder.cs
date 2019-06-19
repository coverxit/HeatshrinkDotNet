using System;
using System.Collections.Generic;

namespace HeatshrinkDotNet
{
    public enum DecoderSinkResult
    {
        Ok = 0,
		Full = 1,
		Null = -1,
	}

    public enum DecoderPollResult
    {
        Empty = 0,
		More = 1,
		Null = -1,
		Unknown = -2
	}

    public enum DecoderFinishResult
    {
        Done = 0,
		More = 1,
		Null = -1
	}

    enum DecoderState
    {
        TagBit,                 // Tag bit
        YieldLiteral,           // Ready to yield literal byte
        BackrefIndexMSB,        // Most significant byte of index
        BackrefIndexLSB,        // Least significant byte of index
        BackrefCountMSB,        // Most significant byte of count
        BackrefCountLSB,        // Least significant byte of count
        YieldBackref,           // Ready to yield back-reference
    }

    public class HeatshrinkDecoder
    {
        #region Constants
        readonly IReadOnlyDictionary<DecoderState, string> StateNames = new Dictionary<DecoderState, string>
        {
            { DecoderState.TagBit,          "tag_bit"           },
            { DecoderState.YieldLiteral,    "yield_literal"     },
            { DecoderState.BackrefIndexMSB, "backref_index_msb" },
            { DecoderState.BackrefIndexLSB, "backref_index_lsb" },
            { DecoderState.BackrefCountMSB, "backref_count_msb" },
            { DecoderState.BackrefCountLSB, "backref_count_lsb" },
            { DecoderState.YieldBackref,    "yield_backref"     }
        };
        #endregion

        #region Internal Variables
        int _inputSize;         // Bytes in input buffer
        int _inputIndex;        // Offset to next unprocessed input byte
        int _outputCount;       // How many bytes to output
        int _outputIndex;       // Index for bytes to output
        int _headIndex;         // Head of window buffer
        DecoderState _state;    // Current state machine node
        byte _currentByte;      // Current byte of input
        byte _bitIndex;         // Curent bit index
        int _windowSz2;         // Window buffer bits
        int _lookaheadSz2;      // Lookahead bits
        int _inputBufferSize;   // Input buffer size
        
        byte[] _buffer;         // Input buffer, then expansion window buffer
        #endregion

        /// <summary>
        /// Allocate a decoder with an input buffer of inputBufferSize bytes,
        /// an expansion buffer size of 2 ^ windowBits, and a lookahead
        /// size of 2 ^ lookaheadBits. (The window buffer and lookahead sizes
        /// must match the settings used when the data was compressed.)
        /// </summary>
        /// <param name="inputBufferSize">Input buffer size</param>
        /// <param name="windowBits">Window size in bits</param>
        /// <param name="lookaheadBits">Lookahead size in bits</param>
        public HeatshrinkDecoder(int inputBufferSize, int windowBits, int lookaheadBits)
        {
            if (windowBits < Constants.MinWindowBits ||
                windowBits > Constants.MaxWindowBits ||
                inputBufferSize <= 0 ||
                lookaheadBits < Constants.MinLookaheadBits ||
                lookaheadBits >= windowBits)
                throw new ArgumentOutOfRangeException();

            var bufferSz = (1 << windowBits) + inputBufferSize;
            _buffer = new byte[bufferSz];

            _inputBufferSize = inputBufferSize;
            _windowSz2 = windowBits;
            _lookaheadSz2 = lookaheadBits;
            Reset();

            Logging.WriteLine($"-- allocated decoder with buffer size of {bufferSz} ({1 << windowBits} + {_inputBufferSize})");
        }

        /// <summary>
        /// Reset the decoder
        /// </summary>
        public void Reset()
        {
            Array.Clear(_buffer, 0, _buffer.Length);
            _state = DecoderState.TagBit;
            _inputSize = 0;
            _inputIndex = 0;
            _bitIndex = 0x00;
            _currentByte = 0x00;
            _outputCount = 0;
            _outputIndex = 0;
            _headIndex = 0;
        }

        /// <summary>
        /// Sink the whole buffer into the decoder.
        /// inputSize is set to the number of bytes actually sunk (in case a buffer was filled)
        /// </summary>
        /// <param name="buffer">The buffer to be filled into the decoder</param>
        /// <param name="inputSize">Number of bytes actually sunk</param>
        /// <returns>Refer to HSE_sink_res in original C version heatshrink</returns>
        public DecoderSinkResult Sink(byte[] buffer, out int inputSize) => Sink(buffer, 0, buffer.Length, out inputSize);

        /// <summary>
        /// Sink up to size bytes from buffer into the decoder.
        /// inputSize is set to the number of bytes actually sunk (in case a buffer was filled)
        /// </summary>
        /// <param name="buffer">The buffer to be filled into the decoder</param>
        /// <param name="size">Number of bytes to be filled</param>
        /// <param name="inputSize">Number of bytes actually sunk</param>
        /// <returns>Refer to HSE_sink_res in original C version heatshrink</returns>
        public DecoderSinkResult Sink(byte[] buffer, int size, out int inputSize) => Sink(buffer, 0, size, out inputSize);

        /// <summary>
        /// Sink up to size bytes from buffer + offset into the decoder.
        /// inputSize is set to the number of bytes actually sunk (in case a buffer was filled)
        /// </summary>
        /// <param name="buffer">The buffer to be filled into the decoder</param>
        /// <param name="offset">The offset of the buffer to be filled</param>
        /// <param name="size">Number of bytes to be filled</param>
        /// <param name="inputSize">Number of bytes actually sunk</param>
        /// <returns>Refer to HSE_sink_res in original C version heatshrink</returns>
        public DecoderSinkResult Sink(byte[] buffer, int offset, int size, out int inputSize)
        {
            inputSize = 0;

            if (buffer == null) return DecoderSinkResult.Null;

            var rem = _inputBufferSize - _inputSize;
            if (rem == 0) return DecoderSinkResult.Full;

            size = rem < size ? rem : size;
            Logging.WriteLine($"-- sinking {size} bytes");
            Array.Copy(buffer, offset, _buffer, _inputSize, size);
            _inputSize += size;
            inputSize = size;
            return DecoderSinkResult.Ok;
        }

        /// <summary>
        /// Poll for output from the decoder, copying up to buffer's length bytes into buffer
        /// (setting outputSize to the actual amount copied)
        /// </summary>
        /// <param name="buffer">The buffer to be filled from the decoder</param>
        /// <param name="outputSize">Number of bytes actually polled</param>
        /// <returns>Refer to HSE_poll_res in original C version heatshrink</returns>
        public DecoderPollResult Poll(byte[] buffer, out int outputSize) => Poll(buffer, 0, buffer.Length, out outputSize);

        /// <summary>
        /// Poll for output from the decoder, copying at most size bytes into buffer
        /// (setting outputSize to the actual amount copied)
        /// </summary>
        /// <param name="buffer">The buffer to be filled from the decoder</param>
        /// <param name="size">Number of bytes to be filled</param>
        /// <param name="outputSize">Number of bytes actually polled</param>
        /// <returns>Refer to HSE_poll_res in original C version heatshrink</returns>
        public DecoderPollResult Poll(byte[] buffer, int size, out int outputSize) => Poll(buffer, 0, size, out outputSize);


        /// <summary>
        /// Poll for output from the decoder, copying at most size bytes into buffer + offset
        /// (setting outputSize to the actual amount copied)
        /// </summary>
        /// <param name="buffer">The buffer to be filled from the decoder</param>
        /// <param name="offset">The offset of the buffer to be filled</param>
        /// <param name="size">Number of bytes to be filled</param>
        /// <param name="outputSize">Number of bytes actually polled</param>
        /// <returns>Refer to HSE_poll_res in original C version heatshrink</returns>
        public DecoderPollResult Poll(byte[] buffer, int offset, int size, out int outputSize)
        {
            outputSize = 0;

            if (buffer == null) return DecoderPollResult.Null;

            while (true)
            {
                Logging.WriteLine($"-- poll, state is {Convert.ToInt32(_state)} ({StateNames[_state]}), input_size {_inputSize}");

                var state = _state;
                switch (state)
                {
                    case DecoderState.TagBit:
                        _state = StTagBit();
                        break;

                    case DecoderState.YieldLiteral:
                        _state = StYieldLiteral(buffer, offset, size, ref outputSize);
                        break;

                    case DecoderState.BackrefIndexMSB:
                        _state = StBackrefIndexMSB();
                        break;

                    case DecoderState.BackrefIndexLSB:
                        _state = StBackrefIndexLSB();
                        break;

                    case DecoderState.BackrefCountMSB:
                        _state = StBackrefCountMSB();
                        break;

                    case DecoderState.BackrefCountLSB:
                        _state = StBackrefCountLSB();
                        break;

                    case DecoderState.YieldBackref:
                        _state = StYieldBackref(buffer, offset, size, ref outputSize);
                        break;

                    default:
                        return DecoderPollResult.Unknown;
                }

                /* If the current state cannot advance, check if input or output
                 * buffer are exhausted. */
                if (_state == state)
                    return outputSize == size ? DecoderPollResult.More : DecoderPollResult.Empty;
            }
        }

        /// <summary>
        /// Notify the decoder that the input stream is finished.
        /// If the return value is HSER_FINISH_MORE, there is still more output, so call poll() and repeat
        /// </summary>
        /// <returns>Refer to HSE_finish_res in original C version heatshrink</returns>
        public DecoderFinishResult Finish()
        {
            switch (_state)
            {
                case DecoderState.TagBit:
                    return _inputSize == 0 ? DecoderFinishResult.Done : DecoderFinishResult.More;

                /* If we want to finish with no input, but are in these states, it's
                 * because the 0-bit padding to the last byte looks like a backref
                 * marker bit followed by all 0s for index and count bits. */
                case DecoderState.BackrefIndexLSB:
                case DecoderState.BackrefIndexMSB:
                case DecoderState.BackrefCountLSB:
                case DecoderState.BackrefCountMSB:
                    return _inputSize == 0 ? DecoderFinishResult.Done : DecoderFinishResult.More;

                /* If the output stream is padded with 0xFFs (possibly due to being in
                 * flash memory), also explicitly check the input size rather than
                 * uselessly returning MORE but yielding 0 bytes when polling. */
                case DecoderState.YieldLiteral:
                    return _inputSize == 0 ? DecoderFinishResult.Done : DecoderFinishResult.More;

                default:
                    return DecoderFinishResult.More;
            }
        }

        #region Helper
        /* Get the next COUNT bits from the input buffer, saving incremental progress.
         * Returns NO_BITS on end of input, or if more than 15 bits are requested. */
        int GetBits(int count)
        {
            var accumulator = 0;

            if (count > 15) return -1;
            Logging.WriteLine($"-- popping {count} bit(s)");

            /* If we aren't able to get COUNT bits, suspend immediately, because we
             * don't track how many bits of COUNT we've accumulated before suspend. */
            if (_inputSize == 0)
                if (_bitIndex < (1 << (count - 1)))
                    return -1;

            for (var i = 0; i < count; i++)
            {
                if (_bitIndex == 0x00)
                {
                    if (_inputSize == 0)
                    {
                        Logging.WriteLine($"  -- out of bits, suspending w/ accumulator of {accumulator} (0x{accumulator:x2})");
                        return -1;
                    }

                    _currentByte = _buffer[_inputIndex++];
                    Logging.WriteLine($"  -- pulled byte 0x{_currentByte:x2}");
                    if (_inputIndex == _inputSize)
                    {
                        _inputIndex = 0; // input is exhausted
                        _inputSize = 0;
                    }
                    _bitIndex = 0x80;
                }

                accumulator <<= 1;
                if (Convert.ToBoolean(_currentByte & _bitIndex))
                    accumulator |= 0x01;
                _bitIndex >>= 1;
            }

            if (count > 1) Logging.WriteLine($"  -- accumulated {accumulator:x8}");
            return accumulator;
        }

        void PushByte(byte[] buffer, int offset, int size, ref int outputSize, byte _byte)
        {
            var _c = Convert.ToChar(_byte);
            _c = !char.IsControl(_c) || char.IsWhiteSpace(_c) ? _c : '.';
            
            Logging.WriteLine($" -- pushing byte: 0x{_byte:x2} ('{_c}')");
            buffer[offset + (outputSize++)] = _byte;
        }
        #endregion

        #region Decompression
        DecoderState StTagBit()
        {
            var bits = GetBits(1); // Get tag bit
            if (bits == -1)
                return DecoderState.TagBit;
            else if (Convert.ToBoolean(bits))
                return DecoderState.YieldLiteral;
            else if (_windowSz2 > 8)
                return DecoderState.BackrefIndexMSB;
            else
            {
                _outputIndex = 0;
                return DecoderState.BackrefIndexLSB;
            }
        }

        DecoderState StYieldLiteral(byte[] buffer, int offset, int size, ref int outputSize)
        {
            /* Emit a repeated section from the window buffer, and add it (again)
             * to the window buffer. (Note that the repetition can include
             * itself.)*/
            if (outputSize < size)
            {
                var _byte = GetBits(8);
                if (_byte == -1) return DecoderState.YieldLiteral; // Out of input

                var bufPos = _inputBufferSize;
                var mask = (1 << _windowSz2) - 1;
                var c = Convert.ToByte(_byte & 0xFF);

                var _c = Convert.ToChar(c);
                _c = !char.IsControl(_c) || char.IsWhiteSpace(_c) ? _c : '.';
                Logging.WriteLine($"-- emitting literal byte 0x{c:x2} ('{_c}')");

                _buffer[bufPos + (_headIndex++ & mask)] = c;
                PushByte(buffer, offset, size, ref outputSize, c);
                return DecoderState.TagBit;
            }
            else
            {
                return DecoderState.YieldLiteral;
            }
        }

        DecoderState StBackrefIndexMSB()
        {
            var bitCt = _windowSz2;
            var bits = GetBits(bitCt - 8);
            Logging.WriteLine($"-- backref index (msb), got 0x{bits:x4} (+1)");

            if (bits == -1) return DecoderState.BackrefIndexMSB;
            _outputIndex = bits << 8;
            return DecoderState.BackrefIndexLSB;
        }

        DecoderState StBackrefIndexLSB()
        {
            var bitCt = _windowSz2;
            var bits = GetBits(bitCt < 8 ? bitCt : 8);
            Logging.WriteLine($"-- backref index (lsb), got 0x{bits:x4} (+1)");

            if (bits == -1) return DecoderState.BackrefIndexLSB;
            _outputIndex |= bits;
            _outputIndex++;

            var brBitCt = _lookaheadSz2;
            _outputCount = 0;
            return brBitCt > 8 ? DecoderState.BackrefCountMSB : DecoderState.BackrefCountLSB;
        }

        DecoderState StBackrefCountMSB()
        {
            var brBitCt = _lookaheadSz2;
            var bits = GetBits(brBitCt - 8);
            Logging.WriteLine($"-- backref count (msb), got 0x{bits:x4} (+1)");

            if (bits == -1) return DecoderState.BackrefCountMSB;
            _outputCount = bits << 8;
            return DecoderState.BackrefCountLSB;
        }

        DecoderState StBackrefCountLSB()
        {
            var brBitCt = _lookaheadSz2;
            var bits = GetBits(brBitCt < 8 ? brBitCt : 8);
            Logging.WriteLine($"-- backref count (lsb), got 0x{bits:x4} (+1)");

            if (bits == -1) return DecoderState.BackrefCountLSB;
            _outputCount |= bits;
            _outputCount++;
            return DecoderState.YieldBackref;
        }

        DecoderState StYieldBackref(byte[] buffer, int offset, int size, ref int outputSize)
        {
            var count = size - outputSize;
            if (count > 0)
            {
                if (_outputCount < count) count = _outputCount;
                var bufPos = _inputBufferSize;
                var mask = (1 << _windowSz2) - 1;
                var negOffset = _outputIndex;
                Logging.WriteLine($"-- emitting ${count} bytes from -{negOffset} bytes back");

                for (var i = 0; i < count; i++)
                {
                    var c = _buffer[bufPos + ((_headIndex - negOffset) & mask)];
                    PushByte(buffer, offset, size, ref outputSize, c);
                    _buffer[bufPos + (_headIndex & mask)] = c;
                    _headIndex++;
                    Logging.WriteLine($"  -- ++ 0x{c:x2}");
                }

                _outputCount -= count;
                if (_outputCount == 0) return DecoderState.TagBit;
            }
            return DecoderState.YieldBackref;
        }
        #endregion
    }
}
