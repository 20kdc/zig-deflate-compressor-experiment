// zig-deflate-compressor - Zig zero-allocation DEFLATE (RFC 1951) Compressor
// Written starting in 2021 by contributors
// To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide. This software is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

const std = @import("std");


// So before we continue, some specific notes about the format.
// + The world in which DEFLATE operates in is a bitstream.
//   The lowest bit of each byte is the first bit in the stream, going forwards.
//   Note however that DEFLATE can forcibly align the stream to an 8-bit boundary in some situations.
// + Most integer fields are little-endian bit-wise, such that an aligned little-endian integer field is stored 'as-is'.
// + Huffman codes are 'effectively' big-endian bit-wise.
//   Some may consider this arguably a detail of how I like to represent Huffman codes,
//    but my logic can be explained by understanding the algorithm that builds a Huffman tree from bit lengths.
//   This algorithm is based around numbers, and the resulting codes following it in this manner are in this 'big-endian' form.
//   It therefore follows that this is the canonical form for calculating the Huffman codes.
//   DEFLATE has a maximum Huffman code length of 15,
//   which gives room in a 16-bit integer for a 1-bit 'always there' prefix to indicate length when implementing the Huffman search via, say, a 65536-entry lookup table.
//   So in a decompressor, you start with 1 (the empty bitstring), and shift it left adding in new incoming bits at the right end.
//   In a compressor, you simply have a mapping from symbols to integers, with separate lengths.

// And about the compressor & fixed huffman format:
// + This compressor only uses Fixed Huffman blocks, as they're easy to write while giving some compression.
// + The literal tree uses the following lengths:
//    for lI = 0, 143 do codeLen[sym] = 8 end
//    for lI = 144, 255 do codeLen[sym] = 9 end
//    for lI = 256, 279 do codeLen[sym] = 7 end
//    for lI = 280, 287 do codeLen[sym] = 8 end
//   What this calculated to is [TABLE MOVED TO writeFxhLiteralSymbol]
// + The distance tree is 32 symbols all of length 5, effectively equivalent to a 1:1 "just write 5 bits" mapping.
//   HOWEVER, the bits are reversed as per the difference between Huffman and non-Huffman integers above.
//   Note that the last 2 symbols of the distance tree will never occur in the compressed data.
//   (This implies they're padding.)

// -- Main Body --

const DEFLATE_LENGTH_BITBASE = struct {
    // Table of extension bit counts for given length symbols, starting with 257.
    const BIT: []const u8 = &[29]u8{
        0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 1, 2, 2, 2, 2,
        3, 3, 3, 3, 4, 4, 4, 4,
        5, 5, 5, 5, 0
    };
    // Table of base lengths for given length symbols, starting with 257.
    // First value here implies minimum length.
    const BASE: []const u16 = &[29]u16{
        3, 4, 5, 6, 7, 8, 9, 10,
        11, 13, 15, 17, 19, 23, 27, 31,
        35, 43, 51, 59, 67, 83, 99, 115,
        131, 163, 195, 227, 258
    };
};

const DEFLATE_DIST_BITBASE = struct {
    // Table of extension bit counts for given distance symbols
    const BIT: []const u8 = &[30]u8{
        0, 0, 0, 0, 1, 1, 2, 2,
        3, 3, 4, 4, 5, 5, 6, 6,
        7, 7, 8, 8, 9, 9, 10, 10,
        11, 11, 12, 12, 13, 13
    };
    // Table of base distances for given distance symbols
    const BASE: []const u16 = &[30]u16{
        1, 2, 3, 4, 5, 7, 9, 13,
        17, 25, 33, 49, 65, 97, 129, 193,
        257, 385, 513, 769, 1025, 1537, 2049, 3073,
        4097, 6145, 8193, 12289, 16385, 24577
    };
};

// Note: it's assumed that bitbases are in increasing order.
// If not, this won't work.
fn chooseBitBase(val: u16, comptime bitbase: type) usize {
    for (bitbase.BASE) |base, idx| {
        var bit = bitbase.BIT[idx];
        var extent = base + (@as(u16, 1) << @truncate(u4, bit));
        if ((val >= base) and (val < extent)) {
            return idx;
        }
    }
    std.debug.panic("failed to find {}", .{ val });
}

pub fn DeflateCompressor(comptime WindowType: type, comptime WriterType: type) type {
    return struct {
        const Window = WindowType;
        forward_writer: BitWriterType,
        window: Window,
        const BitWriterType = std.io.BitWriter(std.builtin.Endian.Little, WriterType);
        const Self = @This();
        // NOTE: This type does not perform any dynamic allocation.
        // This operation inherently cannot fail (no writing is performed yet).
        pub fn init(src: WriterType) Self {
            return Self {
                .forward_writer = BitWriterType.init(src),
                .window = Window.init()
            };
        }

        // Internal: Write symbol, fixed-huffman-table: literal
        fn writeFxhLiteralSymbol(self: *Self, sym: u9) BitWriterType.Error!void {
            if (sym < 144) {
                //    Symbols 0, 143 inc. are 8-bit, base 48
                try self.forward_writer.writeBits(@bitReverse(u8, @truncate(u8, sym + 48)), 8);
            } else if (sym < 256) {
                //    Symbols 144, 255 inc. are 9-bit, base 400
                try self.forward_writer.writeBits(@bitReverse(u9, @truncate(u9, sym + 256)), 9);
            } else if (sym < 280) {
                //    Symbols 256, 279 inc. are 7-bit, base 0
                try self.forward_writer.writeBits(@bitReverse(u7, @truncate(u7, sym - 256)), 7);
            } else {
                //    Symbols 280, 287 inc. are 8-bit, base 192
                try self.forward_writer.writeBits(@bitReverse(u8, @truncate(u8, sym - 88)), 8);
            }
        }

        // Internal: Write symbol, fixed-huffman-table: distance
        fn writeFxhDistanceSymbol(self: *Self, sym: u5) BitWriterType.Error!void {
            try self.forward_writer.writeBits(@bitReverse(u5, sym), 5);
        }

        // This opens a fixed-huffman chunk.
        // Operations not intended to be used in this mode will have terrible effects if they are used anyway.
        // Note that final must be the same between open & close calls.
        // Finally, the state of the DEFLATE stream is undefined if any error occurs.
        // This type will never generate an error that was not received from the underlying writers.
        pub fn openFixedChunk(self: *Self, final: bool) BitWriterType.Error!void {
            // Final block flag.
            try self.forward_writer.writeBits(@boolToInt(final), 1);
            // Fixed Huffman block
            try self.forward_writer.writeBits(@as(u2, 1), 2);
        }

        // This closes a fixed-huffman chunk.
        // Note that final must be the same between open & close calls.
        // Finally, the state of the DEFLATE stream is undefined if any error occurs.
        // This type will never generate an error that was not received from the underlying writers.
        pub fn closeFixedChunk(self: *Self, final: bool) BitWriterType.Error!void {
            try self.writeFxhLiteralSymbol(256);
            if (final) {
                try self.forward_writer.flushBits();
            }
        }

        // Writes an atomic unit of data into an active chunk.
        // This will only consume part of the data given (that part which represents this small atomic unit).
        // Returns the amount remaining, allowing you to refill it up to the window size (this allows for perfect streaming compression)
        // This is only valid in an openFixedChunk-closeFixedChunk pair.
        // Finally, the state of the DEFLATE stream is undefined if any error occurs.
        // This type will never generate an error that was not received from the underlying writers.
        pub fn writeFixedUnit(self: *Self, chunk: []const u8) BitWriterType.Error![]const u8 {
            // Find...
            var find = self.window.find(chunk);
            var successChunk: []const u8 = &[0]u8{};
            // Note the use of DEFLATE_LENGTH_BITBASE here ; the minimum length is important.
            var hasDLPair: bool = find.length >= DEFLATE_LENGTH_BITBASE.BASE[0];
            if (hasDLPair) {
                // std.log.err("has DL pair", .{});
                // Verify pair is probably shorter than an equivalent literal.
                // This also does the setup for the actual writing.
                // If it is shorter, then the writing is actually done here.
                // For reference, u12 will always be correct:
                //  + max length is 258
                //  + max lit. bits is 9
                //  + therefore max size is 2322
                // Work out equivalent literal size.
                // This doesn't account for potential future finds,
                //  but that's not really likely to be a problem.
                var litBits: u12 = 0;
                for (chunk[0..find.length]) |b| {
                    if (b < 144) {
                        litBits += 8;
                    } else {
                        litBits += 9;
                    }
                }
                // Length index
                var lenIdx = chooseBitBase(find.length, DEFLATE_LENGTH_BITBASE);
                var lenSym = @intCast(u9, lenIdx) + 257;
                // Distance index
                var dstIdx = @intCast(u5, chooseBitBase(find.distance, DEFLATE_DIST_BITBASE));
                // Using those indexes, calculate bits..
                var lenSymBits: u12 = 8;
                if (lenSym < 280) {
                    lenSymBits = 7;
                }
                var dpBits: u12 = lenSymBits + DEFLATE_LENGTH_BITBASE.BIT[lenIdx] + 5 + DEFLATE_DIST_BITBASE.BIT[dstIdx];
                if (dpBits < litBits) {
                    // Length-Distance pair
                    // Length & extension...
                    try self.writeFxhLiteralSymbol(lenSym);
                    try self.forward_writer.writeBits(find.length - DEFLATE_LENGTH_BITBASE.BASE[lenIdx], DEFLATE_LENGTH_BITBASE.BIT[lenIdx]);
                    // Distance & extension...
                    try self.writeFxhDistanceSymbol(dstIdx);
                    try self.forward_writer.writeBits(find.distance - DEFLATE_DIST_BITBASE.BASE[dstIdx], DEFLATE_DIST_BITBASE.BIT[dstIdx]);
                    // Actually advance
                    successChunk = chunk[0..find.length];
                    // std.log.err("was executed, L{} - '{s}'", .{find.length, successChunk});
                } else {
                    // Not worth it
                    hasDLPair = false;
                }
            }
            if (!hasDLPair) {
                // DL pair failed - write literal
                try self.writeFxhLiteralSymbol(@intCast(u9, chunk[0]));
                successChunk = chunk[0..1];
                // std.log.err("was lit X {}", .{chunk[0]});
            }
            self.window.inject(successChunk);
            return chunk[successChunk.len..];
        }

        // Writes a chunk. You should make chunks as big as possible, or you will waste resources.
        // In particular, this operation:
        // + Opens a new fixed-huffman block
        // + Writes the entirety of the chunk to the block
        // + Closes the Huffman block
        // The block may be marked final, which terminates the DEFLATE stream.
        // Setting final causes an automatic flush of the underlying BitWriter as no further data should be written after this point.
        // Writing data after a chunk in which the final flag has been set may cause corruption depending on the mode of operation of the framing format.
        // Finally, the state of the DEFLATE stream is undefined if any error occurs.
        // This type will never generate an error that was not received from the underlying writers.
        pub fn write(self: *Self, chunk: []const u8, final: bool) BitWriterType.Error!void {
            try self.openFixedChunk(final);
            var thisChunk: []const u8 = chunk;
            while (thisChunk.len > 0) {
                thisChunk = try self.writeFixedUnit(thisChunk);
            }
            try self.closeFixedChunk(final);
        }

        // Writes a chunk as a literal block (or if required, set of literal blocks)
        // This is not an efficient method of storing data, but it ensures the underlying BitWriter can be (and will have been) flushed,
        //  without decompressor side effects such as needing to be a final block, bitstream corruption, or added output data.
        // It's important to note that passing an empty chunk is a valid way to use this if you for some reason *must* get the stream aligned without terminating it.
        // Writing data after a chunk in which the final flag has been set may cause corruption depending on the mode of operation of the framing format.
        // The final flag may be set here in order to finalize a stream which wasn't previously finalized,
        //  but this is wasteful in comparison to supplying it on the previous block.
        // Finally, the state of the DEFLATE stream is undefined if any error occurs.
        // This type will never generate an error that was not received from the underlying writers.
        pub fn writeLiteral(self: *Self, chunk: []const u8, final: bool) BitWriterType.Error!void {
            var hasWrittenOneBlock: bool = false;
            var thisChunk: []const u8 = chunk;
            while ((!hasWrittenOneBlock) or (thisChunk.len > 0)) {
                var nextChunk: []const u8 = &[0]u8{};
                if (thisChunk.len >= 0x10000) {
                    nextChunk = thisChunk[0xFFFF..];
                    thisChunk = thisChunk[0..0xFFFF];
                }
                var thisFinal = (nextChunk.len == 0) and final;
                // Final block flag.
                try self.forward_writer.writeBits(@boolToInt(thisFinal), 1);
                // Literal block - this is what gets the alignment
                try self.forward_writer.writeBits(@as(u2, 0), 2);
                // Alignment caused by literal block
                try self.forward_writer.flushBits();
                // Literal block length
                const thisChunkLen = @truncate(u16, thisChunk.len);
                try self.forward_writer.writeBits(thisChunkLen, 16);
                // Literal block length inverted
                try self.forward_writer.writeBits(~thisChunkLen, 16);
                // Flush bits just in case.
                try self.forward_writer.flushBits();
                // Time to write contents.
                try self.forward_writer.forward_writer.writeAll(thisChunk);
                // Advance
                hasWrittenOneBlock = true;
                thisChunk = nextChunk;
            }
            // Update window
            self.window.inject(chunk);
        }
    };
}

