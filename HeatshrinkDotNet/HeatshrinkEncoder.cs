using System;
using System.Collections.Generic;
using System.Linq;

namespace HeatshrinkDotNet
{
    public enum EncoderSinkResult
    {
        Ok = 0,
		Null = -1,
		Misuse = -2
	}

    public enum EncoderPollResult
    {
        Empty = 0,
		More = 1,
		Null = -1,
		Misuse = -2
	}

    public enum EncoderFinishResult
    {
        Done = 0,
		More = 1,
		Null = -1
	}

    enum EncoderState
    {
        NotFull,                    // Input buffer not full enough
        Filled,                     // Buffer is full
        Search,                     // Searching for patterns
        YieldTagBit,                // Yield tag bit
        YieldLiteral,               // Emit literal byte
        YieldBackrefIndex,          // Yielding backref index
        YieldBackrefLength,         // Yielding backref length
        SaveBacklog,                // Copying buffer to backlog
        FlushBits,                  // Flush bit buffer
        Done                        // Done
    }

    public class HeatshrinkEncoder
    {
        #region Constants
        const byte FlagIsFinishing = 0x01;
        readonly IReadOnlyDictionary<EncoderState, string> StateNames = new Dictionary<EncoderState, string>
        {
            { EncoderState.NotFull,             "not_full"          },
            { EncoderState.Filled,              "filled"            },
            { EncoderState.Search,              "search"            },
            { EncoderState.YieldTagBit,         "yield_tag_bit"     },
            { EncoderState.YieldLiteral,        "yield_literal"     },
            { EncoderState.YieldBackrefIndex,   "yield_br_index"    },
            { EncoderState.YieldBackrefLength,  "yield_br_length"   },
            { EncoderState.SaveBacklog,         "save_backlog"      },
            { EncoderState.FlushBits,           "flush_bits"        },
            { EncoderState.Done,                "done"              }
        };
        #endregion

        #region Internal Variables
        int _inputSize;             // bytes in input buffer
        int _matchScanIndex;
        int _matchLength;
        int _matchPos;
        int _outgoingBits;          // Enqueued outgoing bits
        int _outgoingBitsCount;
        byte _flags;
        EncoderState _state;        // Current state machine node
        byte _currentByte;          // Current byte of output
        byte _bitIndex;             // Current bit index
        int _windowSz2;             // 2^n size of window
        int _lookaheadSz2;          // 2^n size of lookahead

        int[] _searchIndex;
        byte[] _buffer;             // Input buffer and / sliding window for expansion
        #endregion

        /// <summary>
        /// Allocate a encoder with an expansion buffer size of 2 ^ windowBits, 
        /// and a lookahead size of 2 ^ lookaheadBits
        /// </summary>
        /// <param name="windowBits">Window size in bits</param>
        /// <param name="lookaheadBits">Lookahead size in bits</param>
        public HeatshrinkEncoder(int windowBits, int lookaheadBits)
        {
            if (windowBits < Constants.MinWindowBits || 
                windowBits > Constants.MaxWindowBits || 
                lookaheadBits < Constants.MinLookaheadBits || 
                lookaheadBits >= windowBits)
                throw new ArgumentOutOfRangeException();

            /* Note: 2 * the window size is used because the buffer needs to fit
             * (1 << window_sz2) bytes for the current input, and an additional
             * (1 << window_sz2) bytes for the previous buffer of input, which
             * will be scanned for useful backreferences. */
            var size = 2 << windowBits;

            _buffer = new byte[size];
            _windowSz2 = windowBits;
            _lookaheadSz2 = lookaheadBits;
            Reset();

            _searchIndex = new int[size];

            if (Constants.EnableLogging)
                Console.WriteLine($"-- allocated encoder with buffer size of {size} ({GetInputBufferSize()} byte input size)");
        }

        /// <summary>
		/// Reset the encoder
		/// </summary>
        public void Reset()
        {
            Array.Clear(_buffer, 0, _buffer.Length);
            _inputSize = 0;
            _state = EncoderState.NotFull;
            _matchScanIndex = 0;
            _flags = 0;
            _bitIndex = 0x80;
            _currentByte = 0x00;
            _matchLength = 0;

            _outgoingBits = 0x0000;
            _outgoingBitsCount = 0;
        }

        /// <summary>
		/// Sink up to size bytes from buffer into the encoder.
		/// inputSize is set to the number of bytes actually sunk (in case a buffer was filled)
		/// </summary>
		/// <param name="buffer">The buffer to be filled into the encoder</param>
		/// <param name="size">Number of bytes to be filled</param>
		/// <param name="inputSize">Number of bytes actually sunk</param>
		/// <returns>Refer to HSE_sink_res in original C version heatshrink</returns>
        public EncoderSinkResult Sink(byte[] buffer, out int inputSize) => Sink(buffer, 0, buffer.Length, out inputSize);

        /// <summary>
		/// Sink the whole buffer into the encoder.
		/// inputSize is set to the number of bytes actually sunk (in case a buffer was filled)
		/// </summary>
		/// <param name="buffer">The buffer to be filled into the encoder</param>
		/// <param name="inputSize">Number of bytes actually sunk</param>
		/// <returns>Refer to HSE_sink_res in original C version heatshrink</returns>
        public EncoderSinkResult Sink(byte[] buffer, int size, out int inputSize) => Sink(buffer, 0, size, out inputSize);

        /// <summary>
		/// Sink up to size bytes from buffer + offset into the encoder.
		/// inputSize is set to the number of bytes actually sunk (in case a buffer was filled)
		/// </summary>
		/// <param name="buffer">The buffer to be filled into the encoder</param>
		/// <param name="offset">The offset of the buffer to be filled</param>
		/// <param name="size">Number of bytes to be filled</param>
		/// <param name="inputSize">Number of bytes actually sunk</param>
		/// <returns>Refer to HSE_sink_res in original C version heatshrink</returns>
        public EncoderSinkResult Sink(byte[] buffer, int offset, int size, out int inputSize)
        {
            inputSize = 0;

            if (buffer == null) return EncoderSinkResult.Null;
            
            /* Sinking more content after saying the content is done, tsk tsk */
            if (IsFinishing()) return EncoderSinkResult.Misuse;

            /* Sinking more content before processing is done */
            if (_state != EncoderState.NotFull) return EncoderSinkResult.Misuse;

            var writeOffset = GetInputOffset() + _inputSize;
            var ibs = GetInputBufferSize();
            var rem = ibs - _inputSize;
            var cpSz = rem < size ? rem : size;

            Array.Copy(buffer, offset, _buffer, writeOffset, cpSz);
            inputSize = cpSz;
            _inputSize += cpSz;

            if (Constants.EnableLogging)
                Console.WriteLine($"-- sunk {cpSz} bytes (of {size}) into encoder at {writeOffset}, input buffer now has {_inputSize}");
            if (cpSz == rem)
            {
                if (Constants.EnableLogging) Console.WriteLine("-- internal buffer is now full");
                _state = EncoderState.Filled;
            }
            return EncoderSinkResult.Ok;
        }

        /// <summary>
		/// Poll for output from the encoder, copying up to buffer's length bytes into buffer
		/// (setting outputSize to the actual amount copied)
		/// </summary>
		/// <param name="buffer">The buffer to be filled from the encoder</param>
		/// <param name="outputSize">Number of bytes actually polled</param>
		/// <returns>Refer to HSE_poll_res in original C version heatshrink</returns>
        public EncoderPollResult Poll(byte[] buffer, out int outputSize) => Poll(buffer, 0, buffer.Length, out outputSize);

        /// <summary>
		/// Poll for output from the encoder, copying at most size bytes into buffer
		/// (setting outputSize to the actual amount copied)
		/// </summary>
		/// <param name="buffer">The buffer to be filled from the encoder</param>
		/// <param name="size">Number of bytes to be filled</param>
		/// <param name="outputSize">Number of bytes actually polled</param>
		/// <returns>Refer to HSE_poll_res in original C version heatshrink</returns>
        public EncoderPollResult Poll(byte[] buffer, int size, out int outputSize) => Poll(buffer, 0, size, out outputSize);

        /// <summary>
		/// Poll for output from the encoder, copying at most size bytes into buffer + offset
		/// (setting outputSize to the actual amount copied)
		/// </summary>
		/// <param name="buffer">The buffer to be filled from the encoder</param>
		/// <param name="offset">The offset of the buffer to be filled</param>
		/// <param name="size">Number of bytes to be filled</param>
		/// <param name="outputSize">Number of bytes actually polled</param>
		/// <returns>Refer to HSE_poll_res in original C version heatshrink</returns>
        public EncoderPollResult Poll(byte[] buffer, int offset, int size, out int outputSize)
        {
            outputSize = 0;

            if (buffer == null) return EncoderPollResult.Null;
            if (size == 0)
            {
                if (Constants.EnableLogging) Console.WriteLine("-- MISUSE: output buffer size is 0");
                return EncoderPollResult.Misuse;
            }

            while (true)
            {
                if (Constants.EnableLogging)
                    Console.WriteLine($"-- polling, state {Convert.ToInt32(_state)} ({StateNames[_state]}), flags 0x{_flags:x2}");

                var inState = _state;
                switch (inState)
                {
                    case EncoderState.NotFull:
                        return EncoderPollResult.Empty;

                    case EncoderState.Filled:
                        DoIndexing();
                        _state = EncoderState.Search;
                        break;

                    case EncoderState.Search:
                        _state = StStepSearch();
                        break;

                    case EncoderState.YieldTagBit:
                        _state = StYieldTagBit(buffer, offset, size, ref outputSize);
                        break;

                    case EncoderState.YieldLiteral:
                        _state = StYieldLiteral(buffer, offset, size, ref outputSize);
                        break;

                    case EncoderState.YieldBackrefIndex:
                        _state = StYieldBackrefIndex(buffer, offset, size, ref outputSize);
                        break;

                    case EncoderState.YieldBackrefLength:
                        _state = StYieldBackrefLength(buffer, offset, size, ref outputSize);
                        break;

                    case EncoderState.SaveBacklog:
                        _state = StSaveBacklog();
                        break;

                    case EncoderState.FlushBits:
                        _state = StFlushBitBuffer(buffer, offset, size, ref outputSize);
                        return EncoderPollResult.Empty;

                    case EncoderState.Done:
                        return EncoderPollResult.Empty;

                    default:
                        if (Constants.EnableLogging) Console.WriteLine($"-- bad state {StateNames[_state]}");
                        return EncoderPollResult.Misuse;
                }

                /* Check if output buffer is exhausted. */
                if (_state == inState && outputSize == size) return EncoderPollResult.More;
            }
        }

        /// <summary>
		/// Notify the encoder that the input stream is finished.
		/// If the return value is HSER_FINISH_MORE, there is still more output, so call poll() and repeat
		/// </summary>
		/// <returns>Refer to HSE_finish_res in original C version heatshrink</returns>
        public EncoderFinishResult Finish()
        {
            _flags |= FlagIsFinishing;
            if (Constants.EnableLogging) Console.WriteLine("-- setting is_finishing flag");
            if (_state == EncoderState.NotFull) _state = EncoderState.Filled;
            return _state == EncoderState.Done ? EncoderFinishResult.Done : EncoderFinishResult.More;
        }

        #region Helpers
        int GetInputOffset() => GetInputBufferSize();
        int GetInputBufferSize() => 1 << _windowSz2;
        int GetLookaheadSize() => 1 << _lookaheadSz2;
        bool IsFinishing() => Convert.ToBoolean(_flags & FlagIsFinishing);
        bool CanTakeByte(byte[] buffer, int offset, int size, int outputSize) => outputSize < size;

        void AddTagBit(byte[] buffer, int offset, int size, ref int outputSize, byte tag)
        {
            if (Constants.EnableLogging) Console.WriteLine($"-- adding tag bit: {tag}");
            PushBits(1, tag, buffer, offset, size, ref outputSize);
        }

        /* Push COUNT (max 8) bits to the output buffer, which has room.
         * Bytes are set from the lowest bits, up. */
        void PushBits(int count, int bits, byte[] buffer, int offset, int size, ref int outputSize)
        {
            if (Constants.EnableLogging) Console.WriteLine($"++ push_bits: {count} bits, input of 0x{bits:x2}");

            /* If adding a whole byte and at the start of a new output byte,
             * just push it through whole and skip the bit IO loop. */
            if (count == 8 && _bitIndex == 0x80)
            {
                buffer[offset + (outputSize++)] = (byte) bits;
            }
            else
            {
                for (int i = count - 1; i >= 0; i--)
                {
                    bool bit = Convert.ToBoolean(bits & (1 << i));
                    if (bit) _currentByte |= _bitIndex;

                    _bitIndex >>= 1;
                    if (_bitIndex == 0x00)
                    {
                        _bitIndex = 0x80;
                        if (Constants.EnableLogging) Console.WriteLine($" > pushing byte 0x{_currentByte:x2}");
                        buffer[offset + (outputSize++)] = _currentByte;
                        _currentByte = 0x00;
                    }
                }
            }
        }

        void PushLiteralByte(byte[] buffer, int offset, int size, ref int outputSize)
        {
            var processedOffset = _matchScanIndex - 1;
            var inputOffset = GetInputOffset() + processedOffset;
            var c = _buffer[inputOffset];

            var _c = Convert.ToChar(c);
            _c = !char.IsControl(_c) || char.IsWhiteSpace(_c) ? _c : '.';

            if (Constants.EnableLogging) 
                Console.WriteLine($"-- yielded literal byte 0x{c:x2} ('{_c}') from +{inputOffset}");
            PushBits(8, c, buffer, offset, size, ref outputSize);
        }

        bool PushOutgoingBits(byte[] buffer, int offset, int size, ref int outputSize)
        {
            var count = 0;
            var bits = 0;

            if (_outgoingBitsCount > 8)
            {
                count = 8;
                bits = _outgoingBits >> (_outgoingBitsCount - 8);
            }
            else
            {
                count = _outgoingBitsCount;
                bits = _outgoingBits;
            }

            if (count > 0)
            {
                if (Constants.EnableLogging)
                    Console.WriteLine($"-- pushing {count} outgoing bits: 0x{bits:x2}");
                PushBits(count, bits, buffer, offset, size, ref outputSize);
                _outgoingBitsCount -= count;
            }
            return Convert.ToBoolean(count);
        }
        #endregion

        #region Indexing
        void DoIndexing()
        {
            /* Build an index array I that contains flattened linked lists
             * for the previous instances of every byte in the buffer.
             * 
             * For example, if buf[200] == 'x', then index[200] will either
             * be an offset i such that buf[i] == 'x', or a negative offset
             * to indicate end-of-list. This significantly speeds up matching,
             * while only using sizeof(uint16_t)*sizeof(buffer) bytes of RAM.
             *
             * Future optimization options:
             * 1. Since any negative value represents end-of-list, the other
             *    15 bits could be used to improve the index dynamically.
             *    
             * 2. Likewise, the last lookahead_sz bytes of the index will
             *    not be usable, so temporary data could be stored there to
             *    dynamically improve the index.
             * */
            var last = Enumerable.Repeat(-1, 256).ToArray();

            var inputOffset = GetInputOffset();
            var end = inputOffset + _inputSize;

            for (int i = 0; i < end; i++)
            {
                var v = _buffer[i];
                var lv = last[v];
                _searchIndex[i] = lv;
                last[v] = i;
            }
        }

        int FindLongestMatch(int start, int end, int maxlen, out int matchLength)
        {
            if (Constants.EnableLogging)
                Console.WriteLine($"-- scanning for match of buf[{end}:{end + maxlen}] between buf[{start}:{end + maxlen - 1}] (max {maxlen} bytes)");

            var buf = _buffer;
            var matchMaxLen = 0;
            var matchIndex = -1;

            var len = 0;
            var needlePoint = end;

            var hsi = _searchIndex;
            var pos = _searchIndex[end];

            while (pos - start >= 0)
            {
                var posPoint = pos;
                len = 0;

                /* Only check matches that will potentially beat the current maxlen.
                 * This is redundant with the index if match_maxlen is 0, but the
                 * added branch overhead to check if it == 0 seems to be worse. */

                if (buf[posPoint + matchMaxLen] != buf[needlePoint + matchMaxLen])
                {
                    pos = hsi[pos];
                    continue;
                }

                for (len = 1; len < maxlen; len++)
                    if (buf[posPoint + len] != buf[needlePoint + len])
                        break;

                if (len > matchMaxLen)
                {
                    matchMaxLen = len;
                    matchIndex = pos;
                    if (len == maxlen) break; // won't find better
                }
                pos = hsi[pos];
            }

            var breakEvenPoint = 1 + _windowSz2 + _lookaheadSz2;

            /* Instead of comparing break_even_point against 8*match_maxlen,
             * compare match_maxlen against break_even_point/8 to avoid
             * overflow. Since MIN_WINDOW_BITS and MIN_LOOKAHEAD_BITS are 4 and
             * 3, respectively, break_even_point/8 will always be at least 1. */
            if (matchMaxLen > breakEvenPoint / 8)
            {
                if (Constants.EnableLogging)
                    Console.WriteLine($"-- best match: {matchMaxLen} bytes at -{end - matchIndex}");
                matchLength = matchMaxLen;
                return end - matchIndex;
            }

            if (Constants.EnableLogging) Console.WriteLine("-- none found");
            matchLength = 0;
            return -1;
        }
        #endregion

        #region Compression
        EncoderState StStepSearch()
        {
            var windowLength = GetInputBufferSize();
            var lookaheadSz = GetLookaheadSize();
            var msi = _matchScanIndex;
            if (Constants.EnableLogging)
                Console.WriteLine($"## step_search, scan @ +{msi} ({_inputSize + msi}/{2 * windowLength}), input size {_inputSize}");

            bool fin = IsFinishing();
            if (msi > _inputSize - (fin ? 1 : lookaheadSz))
            {
                /* Current search buffer is exhausted, copy it into the
                 * backlog and await more input. */
                if (Constants.EnableLogging) Console.WriteLine($"-- end of search @ {msi}");
                return fin ? EncoderState.FlushBits : EncoderState.SaveBacklog;
            }

            var inputOffset = GetInputOffset();
            var end = inputOffset + msi;
            var start = end - windowLength;

            var maxPossible = lookaheadSz;
            if (_inputSize - msi < lookaheadSz) maxPossible = _inputSize - msi;

            var matchPos = FindLongestMatch(start, end, maxPossible, out var matchLength);
            if (matchPos == -1)
            {
                if (Constants.EnableLogging) Console.WriteLine("ss Match not found");
                _matchScanIndex++;
                _matchLength = 0;
            }
            else
            {
                if (Constants.EnableLogging) Console.WriteLine($"ss Found match of {matchLength} bytes at {matchPos}");
                _matchPos = matchPos;
                _matchLength = matchLength;
            }

            return EncoderState.YieldTagBit;
        }

        EncoderState StYieldTagBit(byte[] buffer, int offset, int size, ref int outputSize)
        {
            if (CanTakeByte(buffer, offset, size, outputSize))
            {
                if (_matchLength == 0)
                {
                    AddTagBit(buffer, offset, size, ref outputSize, Constants.LiteralMarker);
                    return EncoderState.YieldLiteral;
                }
                else
                {
                    AddTagBit(buffer, offset, size, ref outputSize, Constants.BackrefMarker);
                    _outgoingBits = _matchPos - 1;
                    _outgoingBitsCount = _windowSz2;
                    return EncoderState.YieldBackrefIndex;
                }
            }
            else
            {
                return EncoderState.YieldTagBit; // Output is full, continue
            }
        }

        EncoderState StYieldLiteral(byte[] buffer, int offset, int size, ref int outputSize)
        {
            if (CanTakeByte(buffer, offset, size, outputSize))
            {
                PushLiteralByte(buffer, offset, size, ref outputSize);
                return EncoderState.Search;
            }
            else
            {
                return EncoderState.YieldLiteral;
            }
        }

        EncoderState StYieldBackrefIndex(byte[] buffer, int offset, int size, ref int outputSize)
        {
            if (CanTakeByte(buffer, offset, size, outputSize))
            {
                if (Constants.EnableLogging) Console.WriteLine($"-- yielding backref index {_matchPos}");
                if (PushOutgoingBits(buffer, offset, size, ref outputSize))
                {
                    return EncoderState.YieldBackrefIndex; // continue
                }
                else
                {
                    _outgoingBits = _matchLength - 1;
                    _outgoingBitsCount = _lookaheadSz2;
                    return EncoderState.YieldBackrefLength;
                }
            }
            else
            {
                return EncoderState.YieldBackrefIndex;
            }
        }

        EncoderState StYieldBackrefLength(byte[] buffer, int offset, int size, ref int outputSize)
        {
            if (CanTakeByte(buffer, offset, size, outputSize))
            {
                if (Constants.EnableLogging) Console.WriteLine($"-- yielding backref length {_matchLength}");
                if (PushOutgoingBits(buffer, offset, size, ref outputSize))
                {
                    return EncoderState.YieldBackrefLength;
                }
                else
                {
                    _matchScanIndex += _matchLength;
                    _matchLength = 0;
                    return EncoderState.Search;
                }
            }
            else
            {
                return EncoderState.YieldBackrefLength;
            }
        }

        EncoderState StSaveBacklog()
        {
            var inputBufSize = GetInputBufferSize();
            var msi = _matchScanIndex;

            if (Constants.EnableLogging) Console.WriteLine("-- saving backlog");

            /* Copy processed data to beginning of buffer, so it can be
             * used for future matches. Don't bother checking whether the
             * input is less than the maximum size, because if it isn't,
             * we're done anyway. */
            var rem = inputBufSize - msi;
            var shiftSz = inputBufSize + rem;

            Array.ConstrainedCopy(_buffer, inputBufSize - rem, _buffer, 0, shiftSz);
            _matchScanIndex = 0;
            _inputSize -= inputBufSize - rem;

            return EncoderState.NotFull;
        }

        EncoderState StFlushBitBuffer(byte[] buffer, int offset, int size, ref int outputSize)
        {
            if (_bitIndex == 0x80)
            {
                if (Constants.EnableLogging) Console.WriteLine("-- done");
                return EncoderState.Done;
            }
            else if (CanTakeByte(buffer, offset, size, outputSize))
            {
                if (Constants.EnableLogging)
                    Console.WriteLine($"flushing remaining byte (bit_index == 0x{_bitIndex:x2})");
                buffer[offset + (outputSize++)] = _currentByte;
                return EncoderState.Done;
            }
            else
            {
                if (Constants.EnableLogging) Console.WriteLine("-- done");
                return EncoderState.FlushBits;
            }
        }
        #endregion
    }
}
